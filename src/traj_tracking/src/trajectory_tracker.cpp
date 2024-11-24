#include "trajectory_tracker.h"

#include <Eigen/src/Core/Matrix.h>

#include <cmath>

namespace willand_ackermann {

TrackerParam::TrackerParam(int horizon, double interval, int state_size,
                           int input_size, double speed_limit, double acc_limit,
                           double front_wheel_angle_limit,
                           double front_wheel_angle_rate_limit,
                           double track_width, double dist_front_to_rear)
    : horizon_(horizon),
      interval_(interval),
      state_size_(state_size),
      input_size_(input_size),
      speed_limit_(speed_limit),
      acc_limit_(acc_limit),
      front_wheel_angle_limit_(front_wheel_angle_limit),
      front_wheel_angle_rate_limit_(front_wheel_angle_rate_limit),
      track_width_(track_width),
      dist_front_to_rear_(dist_front_to_rear) {}

TrajectoryTracker::TrajectoryTracker(const TrackerParam &param)
    : param_(param) {
  // all states and inputs are stacked into a large vector
  qp_state_size_ = param_.state_size_ * (param_.horizon_ + 1) +
                   param_.input_size_ * param_.horizon_;
  // pre-allocate matrices memory
  H_.resize(qp_state_size_, qp_state_size_);
  g_.resize(qp_state_size_);
};

bool TrajectoryTracker::update(const DVector &init_state,
                               const Trajectory2D &refer_traj) {
  if (!setReferenceTrajectory(refer_traj)) {
    std::cout << "Invalid reference trajectory!" << std::endl;
    return false;
  }
  if (!setInitialState(init_state)) {
    std::cout << "Invalid initial state!" << std::endl;
    return false;
  }
  setWeightMatrices();
  setGeneralEqualityConstraints();
  setGeneralInequalityConstraints();
  setGeneralBoundBoxConstraints();
  setAccelerateConstraints();
  setSteerRateConstraints();

  // cast everything need
  CastProblemToQpForm();
  return true;
}

bool TrajectoryTracker::solve(DVector &solution) {
  // instantiate the solver
  OsqpEigen::Solver solver;

  // settings
  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);

  // set the initial data of the QP solver
  solver.data()->setNumberOfVariables(qp_state_size_);
  if (!solver.data()->setHessianMatrix(H_)) {
    return false;
  }
  if (!solver.data()->setGradient(g_)) {
    return false;
  }

  solver.data()->setNumberOfConstraints(M_.rows());
  if (!solver.data()->setLinearConstraintsMatrix(M_)) {
    return false;
  }
  if (!solver.data()->setLowerBound(lb_)) {
    return false;
  }
  if (!solver.data()->setUpperBound(ub_)) {
    return false;
  }

  // instantiate the solver
  if (!solver.initSolver()) {
    return false;
  }
  if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
    std::cout << "Osqp solver inner error" << std::endl;
    return false;
  }
  // get the controller input
  solution = solver.getSolution();

  return true;
}

void TrajectoryTracker::CastProblemToQpForm() {
  calcOsqpHession();
  calcOsqpGradient();
  calcOsqpConstraintMatrix();
  calcOsqpConstraintBound();
}

void TrajectoryTracker::calcOsqpHession() {
  // weights for state variables
  DMatrix H = DMatrix::Zero(qp_state_size_, qp_state_size_);
  for (size_t i = 0; i <= param_.horizon_; ++i) {
    H.block(i * param_.state_size_, i * param_.state_size_, param_.state_size_,
            param_.state_size_) = Q_;
  }
  // weights for input variables
  for (size_t i = 0; i < param_.horizon_; ++i) {
    H.block(param_.state_size_ * (param_.horizon_ + 1) + i * param_.input_size_,
            param_.state_size_ * (param_.horizon_ + 1) + i * param_.input_size_,
            param_.input_size_, param_.input_size_) = R_;
  }
  H_ = H.sparseView();
}

void TrajectoryTracker::calcOsqpGradient() {
  /*
   ! Note: the standard form of quadratic programming (QP) problem is as follow
   ! 1/2 * x.transpose() * P * x + q.transpose() * x
   ! that's why the gradient needs to be multiplied by 0.5
  */
  // weights for input variables are all zero
  g_.setZero();
  // weights for state variables
  for (size_t i = 0; i <= param_.horizon_; ++i) {
    g_.segment(i * param_.state_size_, param_.state_size_) =
        -Q_ * refer_state_seq_.at(i);  // has multiplied by 0.5
  }
}
void TrajectoryTracker::calcOsqpConstraintMatrix() {
  // M's rows number = initial state + dynamic model + equality cons +
  // inequality cons + bounding box
  int nums_of_initial_state = param_.state_size_,
      nums_of_dynamic = param_.state_size_ * param_.horizon_,
      nums_of_equality_cons = A_equal_.rows() * param_.horizon_,
      nums_of_inequality_cons = A_inequal_.rows() * param_.horizon_,
      nums_of_state_bounding_box = x_lb_.rows() * (param_.horizon_ + 1),
      nums_of_input_bounding_box = u_lb_.rows() * param_.horizon_,
      nums_of_steer_rate_cons = A_steer_rate_.rows(),
      nums_of_accelerate_cons = A_accelerate_.rows();
  int nums_of_cons_rows = nums_of_initial_state + nums_of_dynamic +
                          nums_of_equality_cons + nums_of_inequality_cons +
                          nums_of_state_bounding_box +
                          nums_of_input_bounding_box + nums_of_steer_rate_cons +
                          nums_of_accelerate_cons;
  //! Note: all the elements in the matrix should be set to zero directly,
  //! otherwise, the matrix will be filled with random values !!!!!!!!
  DMatrix M = DMatrix::Zero(nums_of_cons_rows, qp_state_size_);
  // M.resize(nums_of_cons_rows, qp_state_size_);

  // initial state cons
  M.block(0, 0, param_.state_size_, param_.state_size_).setIdentity();
  // dynamic
  if (nums_of_dynamic != 0) {
    for (size_t i = 0; i < param_.horizon_; ++i) {
      auto &refer_state = refer_state_seq_.at(i);
      auto &refer_input = refer_input_seq_.at(i);
      M.block(nums_of_initial_state + i * param_.state_size_,
              i * param_.state_size_, param_.state_size_, param_.state_size_) =
          dynamicStateMatrixCaster(refer_state, refer_input);
      M.block(nums_of_initial_state + i * param_.state_size_,
              (i + 1) * param_.state_size_, param_.state_size_,
              param_.state_size_) =
          -Eigen::MatrixXd::Identity(param_.state_size_, param_.state_size_);
      M.block(
          nums_of_initial_state + i * param_.state_size_,
          (param_.horizon_ + 1) * param_.state_size_ + i * param_.input_size_,
          param_.state_size_, param_.input_size_) =
          dynamicInputMatrixCaster(refer_state, refer_input);
    }
  }
  // equality cons
  if (nums_of_equality_cons != 0) {
    int block_rows = A_equal_.rows();
    int start_row_offset = nums_of_initial_state + nums_of_dynamic;
    for (size_t i = 0; i < param_.horizon_; ++i) {
      M.block(start_row_offset + i * block_rows, i * param_.state_size_,
              block_rows, param_.state_size_) = A_equal_;
      M.block(
          start_row_offset + i * block_rows,
          (param_.horizon_ + 1) * param_.state_size_ + i * param_.input_size_,
          block_rows, param_.input_size_) = B_equal_;
    }
  }
  // inequality cons
  if (nums_of_inequality_cons != 0) {
    int block_rows = A_inequal_.rows();
    int start_row_offset =
        nums_of_initial_state + nums_of_dynamic + nums_of_equality_cons;
    for (size_t i = 0; i < param_.horizon_; ++i) {
      M.block(start_row_offset + i * block_rows, i * param_.state_size_,
              block_rows, param_.state_size_) = A_inequal_;
      M.block(
          start_row_offset + i * block_rows,
          (param_.horizon_ + 1) * param_.state_size_ + i * param_.input_size_,
          block_rows, param_.input_size_) = B_inequal_;
    }
  }
  // } else {
  //   std::cout << "No inequality constraints need to be casted." << std::endl;
  // }

  // state bounding box cons
  if (nums_of_state_bounding_box != 0) {
    int block_rows = param_.state_size_;
    int start_row_offset = nums_of_initial_state + nums_of_dynamic +
                           nums_of_equality_cons + nums_of_inequality_cons;
    for (size_t i = 0; i <= param_.horizon_; ++i) {
      M.block(start_row_offset + i * block_rows, i * param_.state_size_,
              block_rows, param_.state_size_) =
          Eigen::MatrixXd::Identity(param_.state_size_, param_.state_size_);
    }
  }
  // input bounding box cons
  if (nums_of_input_bounding_box != 0) {
    int block_rows = param_.input_size_;
    int start_row_offset = nums_of_initial_state + nums_of_dynamic +
                           nums_of_equality_cons + nums_of_inequality_cons +
                           nums_of_state_bounding_box;
    for (size_t i = 0; i < param_.horizon_; ++i) {
      M.block(
          start_row_offset + i * block_rows,
          (param_.horizon_ + 1) * param_.state_size_ + i * param_.input_size_,
          block_rows, param_.input_size_) =
          Eigen::MatrixXd::Identity(param_.input_size_, param_.input_size_);
    }
  }
  // steer rate cons
  if (nums_of_steer_rate_cons != 0) {
    int start_row_offset = nums_of_initial_state + nums_of_dynamic +
                           nums_of_equality_cons + nums_of_inequality_cons +
                           nums_of_state_bounding_box +
                           nums_of_input_bounding_box;
    M.block(start_row_offset, 0, A_steer_rate_.rows(), A_steer_rate_.cols()) =
        A_steer_rate_;
  }

  // accelerate cons
  if (nums_of_accelerate_cons != 0) {
    int start_row_offset = nums_of_initial_state + nums_of_dynamic +
                           nums_of_equality_cons + nums_of_inequality_cons +
                           nums_of_state_bounding_box +
                           nums_of_input_bounding_box + nums_of_steer_rate_cons;
    M.block(start_row_offset, 0, A_accelerate_.rows(), A_accelerate_.cols()) =
        A_accelerate_;
  }
  M_ = M.sparseView();
}
void TrajectoryTracker::calcOsqpConstraintBound() {
  int nums_of_initial_state = param_.state_size_,
      nums_of_dynamic = param_.state_size_ * param_.horizon_,
      nums_of_equality_cons = A_equal_.rows() * param_.horizon_,
      nums_of_inequality_cons = A_inequal_.rows() * param_.horizon_,
      nums_of_state_bounding_box = x_lb_.rows() * (param_.horizon_ + 1),
      nums_of_input_bounding_box = u_lb_.rows() * param_.horizon_,
      nums_of_steer_rate_cons = A_steer_rate_.rows(),
      nums_of_accelerate_cons = A_accelerate_.rows();
  int nums_of_cons_rows = nums_of_initial_state + nums_of_dynamic +
                          nums_of_equality_cons + nums_of_inequality_cons +
                          nums_of_state_bounding_box +
                          nums_of_input_bounding_box + nums_of_steer_rate_cons +
                          nums_of_accelerate_cons;
  lb_.resize(nums_of_cons_rows);
  ub_.resize(nums_of_cons_rows);

  // initial state cons
  lb_.segment(0, param_.state_size_) = init_state_;
  ub_.segment(0, param_.state_size_) = init_state_;
  // dynamic cons
  if (nums_of_dynamic != 0) {
    for (size_t i = 0; i < param_.horizon_; ++i) {
      auto &refer_state = refer_state_seq_.at(i);
      auto &refer_input = refer_input_seq_.at(i);
      lb_.segment(nums_of_initial_state + i * param_.state_size_,
                  param_.state_size_) =
          -dynamicVectorCaster(refer_state, refer_input);

      ub_.segment(nums_of_initial_state + i * param_.state_size_,
                  param_.state_size_) =
          -dynamicVectorCaster(refer_state, refer_input);
    }
  }
  // equality cons
  if (nums_of_equality_cons != 0) {
    int block_rows = A_equal_.rows();
    int start_row_offset = nums_of_initial_state + nums_of_dynamic;
    for (size_t i = 0; i < param_.horizon_; ++i) {
      lb_.segment(start_row_offset + i * block_rows, block_rows) = K_equal_;
      ub_.segment(start_row_offset + i * block_rows, block_rows) = K_equal_;
    }
  }
  // inequality cons
  if (nums_of_inequality_cons != 0) {
    int block_rows = A_inequal_.rows();
    int start_row_offset =
        nums_of_initial_state + nums_of_dynamic + nums_of_equality_cons;
    for (size_t i = 0; i < param_.horizon_; ++i) {
      lb_.segment(start_row_offset + i * block_rows, block_rows) =
          K_inequal_lb_;
      ub_.segment(start_row_offset + i * block_rows, block_rows) =
          K_inequal_ub_;
    }
  }
  // state bounding box cons
  if (nums_of_state_bounding_box != 0) {
    int block_rows = param_.state_size_;
    int start_row_offset = nums_of_initial_state + nums_of_dynamic +
                           nums_of_equality_cons + nums_of_inequality_cons;
    for (size_t i = 0; i <= param_.horizon_; ++i) {
      lb_.segment(start_row_offset + i * block_rows, block_rows) = x_lb_;
      ub_.segment(start_row_offset + i * block_rows, block_rows) = x_ub_;
    }
  }
  // input bounding box cons
  if (nums_of_input_bounding_box != 0) {
    int block_rows = param_.input_size_;
    int start_row_offset = nums_of_initial_state + nums_of_dynamic +
                           nums_of_equality_cons + nums_of_inequality_cons +
                           nums_of_state_bounding_box;
    for (size_t i = 0; i < param_.horizon_; ++i) {
      lb_.segment(start_row_offset + i * block_rows, block_rows) = u_lb_;
      ub_.segment(start_row_offset + i * block_rows, block_rows) = u_ub_;
    }
  }
  // steer rate cons
  if (nums_of_steer_rate_cons != 0) {
    int start_row_offset = nums_of_initial_state + nums_of_dynamic +
                           nums_of_equality_cons + nums_of_inequality_cons +
                           nums_of_state_bounding_box +
                           nums_of_input_bounding_box;
    lb_.segment(start_row_offset, lb_steer_rate_.rows()) = lb_steer_rate_;
    ub_.segment(start_row_offset, ub_steer_rate_.rows()) = ub_steer_rate_;
  }
  // accelerate cons
  if (nums_of_accelerate_cons != 0) {
    int start_row_offset = nums_of_initial_state + nums_of_dynamic +
                           nums_of_equality_cons + nums_of_inequality_cons +
                           nums_of_state_bounding_box +
                           nums_of_input_bounding_box + nums_of_steer_rate_cons;
    lb_.segment(start_row_offset, lb_accelerate_.rows()) = lb_accelerate_;
    ub_.segment(start_row_offset, ub_accelerate_.rows()) = ub_accelerate_;
  }
}
bool TrajectoryTracker::setReferenceTrajectory(const Trajectory2D &refer_traj) {
  if (refer_traj.size() <= 1) {
    std::cout << "Invalid reference trajectory" << std::endl;
    return false;
  }

  // given a 2-d trajectory, calculate the reference states and inputs
  // calculate reference states: (x, y, theta)
  refer_state_seq_.clear();
  refer_state_seq_.resize(param_.horizon_ + 1);
  for (size_t i = 0; i < param_.horizon_; ++i) {
    auto &refer_state = refer_state_seq_.at(i);
    refer_state.resize(param_.state_size_);
    refer_state.segment(0, 2) = refer_traj.at(i);
    double delta_x = refer_traj.at(i + 1)(0) - refer_traj.at(i)(0);
    double delta_y = refer_traj.at(i + 1)(1) - refer_traj.at(i)(1);
    double yaw = std::atan(delta_y / (delta_x + kEps));
    yaw = delta_x > 0 ? yaw : delta_y > 0 ? yaw + M_PI : yaw - M_PI;
    // avoid the yaw angle jump
    if (i > 0) {
      double delta_yaw = yaw - refer_state_seq_.at(i - 1)(2);
      int r = std::round(-delta_yaw / (2 * M_PI));
      yaw += r * 2 * M_PI;
    }
    refer_state(2) = yaw;
  }
  auto &refer_state = refer_state_seq_.back();
  refer_state.resize(param_.state_size_);
  refer_state.segment(0, 2) = refer_traj.back();
  refer_state(2) = refer_state_seq_.at(param_.horizon_ - 1)(2);

  // calculate approximate inputs: v and omega
  refer_input_seq_.clear();
  refer_input_seq_.resize(param_.horizon_);
  for (size_t i = 0; i < param_.horizon_; ++i) {
    auto &refer_input = refer_input_seq_.at(i);
    double delta_x = refer_traj.at(i + 1)(0) - refer_traj.at(i)(0);
    double delta_y = refer_traj.at(i + 1)(1) - refer_traj.at(i)(1);
    double v =
        std::sqrt(delta_x * delta_x + delta_y * delta_y) / param_.interval_;
    refer_input.resize(param_.input_size_);
    refer_input(0) = v;
  }
  for (size_t i = 1; i < param_.horizon_; ++i) {
    auto &refer_input = refer_input_seq_.at(i);
    // calculate curvature by three points, assume moving along a circle path
    // in a short distance
    Point2d A = refer_state_seq_.at(i - 1).segment<2>(0);
    Point2d B = refer_state_seq_.at(i).segment<2>(0);
    Point2d C = refer_state_seq_.at(i + 1).segment<2>(0);
    Vector2d ab = B - A, ac = C - A, bc = C - B;
    if (ab.norm() > 1e-6 && bc.norm() > 1e-6) {
      double angle_included =
          std::acos(ab.dot(ac) / (ab.norm() * ac.norm() + kEps));
      double radius =
          std::abs(bc.norm() / (2 * std::sin(angle_included) + kEps));
      double omega = refer_input_seq_.at(i)(0) / (radius + kEps);
      // determine the sign of omega
      if (ab(0) * ac(1) - ab(1) * ac(0) < 0) {
        omega = -omega;
      }
      refer_input(1) = omega;
    } else {
      refer_input(1) = 0.0;
    }
  }
  // the first input vector is not yet be calculated
  refer_input_seq_.front()(1) = refer_input_seq_.at(1)(1);
  return true;
}
TrajectoryTracker::DMatrix TrajectoryTracker::dynamicStateMatrixCaster(
    const DVector &x_refer, const DVector &u_refer) {
  int state_size = param_.state_size_;
  int input_size = param_.input_size_;
  double interval = param_.interval_;
  double theta = x_refer(2), v = u_refer(0);
  DMatrix partial_x(state_size, state_size);
  partial_x << 0, 0, 0, 0, 0, 0, -v * std::sin(theta), v * std::cos(theta), 0;
  return DMatrix::Identity(state_size, state_size) +
         interval * partial_x.transpose();
}
TrajectoryTracker::DMatrix TrajectoryTracker::dynamicInputMatrixCaster(
    const DVector &x_refer, const DVector &u_refer) {
  int state_size = param_.state_size_;
  int input_size = param_.input_size_;
  double interval = param_.interval_;
  double theta = x_refer(2), v = u_refer(0);
  DMatrix partial_u(input_size, state_size);
  partial_u << std::cos(theta), std::sin(theta), 0, 0, 0, 1;
  return interval * partial_u.transpose();
}
TrajectoryTracker::DVector TrajectoryTracker::dynamicVectorCaster(
    const DVector &x_refer, const DVector &u_refer) {
  int state_size = param_.state_size_;
  int input_size = param_.input_size_;
  double interval = param_.interval_;
  double theta = x_refer(2), v = u_refer(0), omega = u_refer(1);
  DVector x_dot(state_size);
  DMatrix partial_x(state_size, state_size);
  DMatrix partial_u(input_size, state_size);
  x_dot << v * std::cos(theta), v * std::sin(theta), omega;
  partial_x << 0, 0, 0, 0, 0, 0, -v * std::sin(theta), v * std::cos(theta), 0;
  partial_u << std::cos(theta), std::sin(theta), 0, 0, 0, 1;
  return interval * (x_dot - partial_x.transpose() * x_refer -
                     partial_u.transpose() * u_refer);
}
TrajectoryTracker::DMatrix
TrajectoryTracker::steerRateConstraintsMatrixCaster() {
  DMatrix P = DMatrix::Zero(2 * (param_.horizon_ - 1), qp_state_size_);
  int wheel_num = 2, state_size = param_.state_size_,
      input_size = param_.input_size_, horizon = param_.horizon_,
      interval = param_.interval_, track_width = param_.track_width_,
      dist_front_to_rear = param_.dist_front_to_rear_,
      qp_state_size = state_size * (horizon + 1) + input_size * horizon;
  for (size_t i = 0; i < horizon - 1; ++i) {
    DVector left_cons = DVector::Zero(qp_state_size);
    double v_0 = refer_input_seq_.at(i)(0), v_1 = refer_input_seq_.at(i + 1)(0),
           omega_0 = refer_input_seq_.at(i)(1),
           omega_1 = refer_input_seq_.at(i + 1)(1);
    double g_0 = (2 * dist_front_to_rear * omega_0) /
                 (2 * v_0 - track_width * omega_0 + kEps),
           g_1 = (2 * dist_front_to_rear * omega_1) /
                 (2 * v_1 - track_width * omega_1 + kEps);
    double partial_g_v_0 = -(4 * dist_front_to_rear * omega_0) /
                           std::pow(2 * v_0 - track_width * omega_0 + kEps, 2),
           partial_g_v_1 = -(4 * dist_front_to_rear * omega_1) /
                           std::pow(2 * v_1 - track_width * omega_1 + kEps, 2),
           partial_g_omega_0 =
               (4 * dist_front_to_rear * v_0) /
               std::pow(2 * v_0 - track_width * omega_0 + kEps, 2),
           partial_g_omega_1 =
               (4 * dist_front_to_rear * v_1) /
               std::pow(2 * v_1 - track_width * omega_1 + kEps, 2);
    double partial_h_v_0 = -(std::pow(g_1, 2) * partial_g_v_0 + partial_g_v_0) /
                           std::pow(1 + g_1 * g_0, 2),
           partial_h_v_1 = (std::pow(g_0, 2) * partial_g_v_1 + partial_g_v_1) /
                           std::pow(1 + g_1 * g_0, 2),
           partial_h_omega_0 =
               -(std::pow(g_1, 2) * partial_g_omega_0 + partial_g_omega_0) /
               std::pow(1 + g_1 * g_0, 2),
           partial_h_omega_1 =
               (std::pow(g_0, 2) * partial_g_omega_1 + partial_g_omega_1) /
               std::pow(1 + g_1 * g_0, 2);
    left_cons((horizon + 1) * state_size + i * input_size) = partial_h_v_0;
    left_cons((horizon + 1) * state_size + (i + 1) * input_size) =
        partial_h_v_1;
    left_cons((horizon + 1) * state_size + i * input_size + 1) =
        partial_h_omega_0;
    left_cons((horizon + 1) * state_size + (i + 1) * input_size + 1) =
        partial_h_omega_1;
    P.row(2 * i) = left_cons.transpose();
  }
  for (size_t i = 0; i < horizon - 1; ++i) {
    DVector right_cons = DVector::Zero(qp_state_size);
    double v_0 = refer_input_seq_.at(i)(0), v_1 = refer_input_seq_.at(i + 1)(0),
           omega_0 = refer_input_seq_.at(i)(1),
           omega_1 = refer_input_seq_.at(i + 1)(1);
    double g_0 = (2 * dist_front_to_rear * omega_0) /
                 (2 * v_0 + track_width * omega_0 + kEps),
           g_1 = (2 * dist_front_to_rear * omega_1) /
                 (2 * v_1 + track_width * omega_1 + kEps);
    double partial_g_v_0 = -(4 * dist_front_to_rear * omega_0) /
                           std::pow(2 * v_0 + track_width * omega_0 + kEps, 2),
           partial_g_v_1 = -(4 * dist_front_to_rear * omega_1) /
                           std::pow(2 * v_1 + track_width * omega_1 + kEps, 2),
           partial_g_omega_0 =
               (4 * dist_front_to_rear * v_0) /
               std::pow(2 * v_0 + track_width * omega_0 + kEps, 2),
           partial_g_omega_1 =
               (4 * dist_front_to_rear * v_1) /
               std::pow(2 * v_1 + track_width * omega_1 + kEps, 2);
    double partial_h_v_0 = -(std::pow(g_1, 2) * partial_g_v_0 + partial_g_v_0) /
                           std::pow(1 + g_1 * g_0, 2),
           partial_h_v_1 = (std::pow(g_0, 2) * partial_g_v_1 + partial_g_v_1) /
                           std::pow(1 + g_1 * g_0, 2),
           partial_h_omega_0 =
               -(std::pow(g_1, 2) * partial_g_omega_0 + partial_g_omega_0) /
               std::pow(1 + g_1 * g_0, 2),
           partial_h_omega_1 =
               (std::pow(g_0, 2) * partial_g_omega_1 + partial_g_omega_1) /
               std::pow(1 + g_1 * g_0, 2);
    right_cons((horizon + 1) * state_size + i * input_size) = partial_h_v_0;
    right_cons((horizon + 1) * state_size + (i + 1) * input_size) =
        partial_h_v_1;
    right_cons((horizon + 1) * state_size + i * input_size + 1) =
        partial_h_omega_0;
    right_cons((horizon + 1) * state_size + (i + 1) * input_size + 1) =
        partial_h_omega_1;
    P.row(2 * i + 1) = right_cons.transpose();
  }
  return P;
}
TrajectoryTracker::DMatrix
TrajectoryTracker::steerRateConstraintsBoundCaster() {
  int horizon = param_.horizon_, state_size = param_.state_size_,
      input_size = param_.input_size_, interval = param_.interval_,
      front_wheel_angle_rate_limit = param_.front_wheel_angle_rate_limit_,
      track_width = param_.track_width_,
      dist_front_to_rear = param_.dist_front_to_rear_,
      qp_state_size = state_size * (horizon + 1) + input_size * horizon;
  DMatrix bound = DMatrix::Zero(2 * (horizon - 1), 2);
  for (size_t i = 0; i < horizon - 1; ++i) {
    double v_0 = refer_input_seq_.at(i)(0), v_1 = refer_input_seq_.at(i + 1)(0),
           omega_0 = refer_input_seq_.at(i)(1),
           omega_1 = refer_input_seq_.at(i + 1)(1);
    double g_0 = (2 * dist_front_to_rear * omega_0) /
                 (2 * v_0 - track_width * omega_0 + kEps),
           g_1 = (2 * dist_front_to_rear * omega_1) /
                 (2 * v_1 - track_width * omega_1 + kEps);
    double partial_g_v_0 = -(4 * dist_front_to_rear * omega_0) /
                           std::pow(2 * v_0 - track_width * omega_0 + kEps, 2),
           partial_g_v_1 = -(4 * dist_front_to_rear * omega_1) /
                           std::pow(2 * v_1 - track_width * omega_1 + kEps, 2),
           partial_g_omega_0 =
               (4 * dist_front_to_rear * v_0) /
               std::pow(2 * v_0 - track_width * omega_0 + kEps, 2),
           partial_g_omega_1 =
               (4 * dist_front_to_rear * v_1) /
               std::pow(2 * v_1 - track_width * omega_1 + kEps, 2);
    double partial_h_v_0 = -(std::pow(g_1, 2) * partial_g_v_0 + partial_g_v_0) /
                           std::pow(1 + g_1 * g_0, 2),
           partial_h_v_1 = (std::pow(g_0, 2) * partial_g_v_1 + partial_g_v_1) /
                           std::pow(1 + g_1 * g_0, 2),
           partial_h_omega_0 =
               -(std::pow(g_1, 2) * partial_g_omega_0 + partial_g_omega_0) /
               std::pow(1 + g_1 * g_0, 2),
           partial_h_omega_1 =
               (std::pow(g_0, 2) * partial_g_omega_1 + partial_g_omega_1) /
               std::pow(1 + g_1 * g_0, 2);
    double k = (g_1 - g_0) / (1 + g_1 * g_0) - partial_h_v_0 * v_0 -
               partial_h_v_1 * v_1 - partial_h_omega_0 * omega_0 -
               partial_h_omega_1 * omega_1;
    bound(2 * i, 0) = -std::tan(interval * front_wheel_angle_rate_limit) - k;
    bound(2 * i, 1) = std::tan(interval * front_wheel_angle_rate_limit) - k;
  }
  for (size_t i = 0; i < horizon - 1; ++i) {
    double v_0 = refer_input_seq_.at(i)(0), v_1 = refer_input_seq_.at(i + 1)(0),
           omega_0 = refer_input_seq_.at(i)(1),
           omega_1 = refer_input_seq_.at(i + 1)(1);
    double g_0 = (2 * dist_front_to_rear * omega_0) /
                 (2 * v_0 + track_width * omega_0 + kEps),
           g_1 = (2 * dist_front_to_rear * omega_1) /
                 (2 * v_1 + track_width * omega_1 + kEps);
    double partial_g_v_0 = -(4 * dist_front_to_rear * omega_0) /
                           std::pow(2 * v_0 + track_width * omega_0 + kEps, 2),
           partial_g_v_1 = -(4 * dist_front_to_rear * omega_1) /
                           std::pow(2 * v_1 + track_width * omega_1 + kEps, 2),
           partial_g_omega_0 =
               (4 * dist_front_to_rear * v_0) /
               std::pow(2 * v_0 + track_width * omega_0 + kEps, 2),
           partial_g_omega_1 =
               (4 * dist_front_to_rear * v_1) /
               std::pow(2 * v_1 + track_width * omega_1 + kEps, 2);
    double partial_h_v_0 = -(std::pow(g_1, 2) * partial_g_v_0 + partial_g_v_0) /
                           std::pow(1 + g_1 * g_0, 2),
           partial_h_v_1 = (std::pow(g_0, 2) * partial_g_v_1 + partial_g_v_1) /
                           std::pow(1 + g_1 * g_0, 2),
           partial_h_omega_0 =
               -(std::pow(g_1, 2) * partial_g_omega_0 + partial_g_omega_0) /
               std::pow(1 + g_1 * g_0, 2),
           partial_h_omega_1 =
               (std::pow(g_0, 2) * partial_g_omega_1 + partial_g_omega_1) /
               std::pow(1 + g_1 * g_0, 2);
    double k = (g_1 - g_0) / (1 + g_1 * g_0) - partial_h_v_0 * v_0 -
               partial_h_v_1 * v_1 - partial_h_omega_0 * omega_0 -
               partial_h_omega_1 * omega_1;
    bound(2 * i + 1, 0) =
        -std::tan(interval * front_wheel_angle_rate_limit) - k;
    bound(2 * i + 1, 1) = std::tan(interval * front_wheel_angle_rate_limit) - k;
  }
  return bound;
}
TrajectoryTracker::DMatrix
TrajectoryTracker::accelerateConstraintsMatrixCaster() {
  DMatrix P = DMatrix::Zero(param_.horizon_ - 1, qp_state_size_);
  int state_size = param_.state_size_, input_size = param_.input_size_,
      horizon = param_.horizon_,
      qp_state_size = state_size * (horizon + 1) + input_size * horizon;
  for (size_t i = 0; i < horizon - 1; ++i) {
    P(i, (horizon + 1) * state_size + i * input_size) = -1;
    P(i, (horizon + 1) * state_size + (i + 1) * input_size) = 1;
  }
  return P;
}
TrajectoryTracker::DMatrix
TrajectoryTracker::accelerateConstraintsBoundCaster() {
  int horizon = param_.horizon_, acc_limit = param_.acc_limit_,
      interval = param_.interval_;
  DMatrix bound = DMatrix::Zero(horizon - 1, 2);
  for (size_t i = 0; i < horizon - 1; ++i) {
    bound(i, 0) = -acc_limit * interval;
    bound(i, 1) = acc_limit * interval;
  }
  return bound;
}
void TrajectoryTracker::printRefereceStateSeq() {
  Eigen::IOFormat CleanFmt(4, Eigen::DontAlignCols, ", ", "", "(", ")");
  for (size_t i = 0; i <= param_.horizon_; ++i) {
    std::cout << std::fixed << std::setprecision(2) << std::setw(4)
              << std::showpos << "time stamp = " << i * param_.interval_
              << ", reference state = "
              << refer_state_seq_.at(i).transpose().format(CleanFmt)
              << std::endl;
  }
}
void TrajectoryTracker::printRefereceInputSeq() {
  Eigen::IOFormat CleanFmt(4, Eigen::DontAlignCols, ", ", "", "(", ")");
  for (size_t i = 0; i < param_.horizon_; ++i) {
    std::cout << std::fixed << std::setprecision(2) << std::setw(4)
              << std::showpos << "time stamp = " << i * param_.interval_
              << ", reference state = "
              << refer_input_seq_.at(i).transpose().format(CleanFmt)
              << std::endl;
  }
}

};  // namespace willand_ackermann
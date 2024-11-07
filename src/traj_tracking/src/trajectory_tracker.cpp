#include "trajectory_tracker.h"

namespace willand_ackermann {

TrajectoryTracker::TrajectoryTracker(int state_size, int input_size,
                                     int horizon, double interval)
    : state_size_(state_size), input_size_(input_size), horizon_(horizon),
      interval_(interval) {
  // all states and inputs are stacked into a large vector
  qp_state_size_ = state_size * (horizon + 1) + input_size * horizon;
  // pre-allocate matrices memory
  H_.resize(qp_state_size_, qp_state_size_);
  g_.resize(qp_state_size_);
};
bool TrajectoryTracker::init(const DMatrix &Q, const DMatrix &R,
                             const DMatrix &Ad, const DMatrix &Bd,
                             const DVector &Kd, const DMatrix &A_equal,
                             const DMatrix &B_equal, const DVector &K_equal,
                             const DMatrix &A_inequal, const DMatrix &B_inequal,
                             const DVector &K_inequal_lb,
                             const DVector &K_inequal_ub, const DVector &x_lb,
                             const DVector &x_ub, const DVector &u_lb,
                             const DVector &u_ub, const DVector &init_state,
                             const Trajectory2D &refer_traj) {
  if (!setInitialState(init_state)) {
    return false;
  }
  if (!setReferenceTrajectory(refer_traj))
    if (!setWeightMatrices(Q, R)) {
      return false;
    }
  if (!setWeightMatrices(Q, R)) {
    return false;
  }
  if (!setDynamicMatrices(Ad, Bd, Kd)) {
    return false;
  }
  if (!setEqualityConstraints(A_equal, B_equal, K_equal)) {
    return false;
  }
  if (!setInequalityConstraints(A_inequal, B_inequal, K_inequal_lb,
                                K_inequal_ub)) {
    return false;
  }
  if (!setBoundBoxConstraints(x_lb, x_ub, u_lb, u_ub)) {
    return false;
  }

  return true;
}
void TrajectoryTracker::calcOsqpHession() {
  // weights for state variables
  for (size_t i = 0; i <= horizon_; ++i) {
    H_.block(i * state_size_, i * state_size_, state_size_, state_size_) = Q_;
  }
  // weights for input variables
  for (size_t i = 0; i < horizon_; ++i) {
    H_.block(state_size_ * (horizon_ + 1) + i * input_size_,
             state_size_ * (horizon_ + 1) + i * input_size_, input_size_,
             input_size_) = R_;
  }
}

void TrajectoryTracker::calcOsqpGradient() {
  // weights for input variables are all zero
  g_.setZero();
  // weights for state variables
  for (size_t i = 0; i <= horizon_; ++i) {
    g_.segment(i * state_size_, state_size_) =
        -2 * Q_.transpose() * refer_traj_ptr_->at(i);
  }
}
void TrajectoryTracker::calcOsqpConstraintMatrix() {
  // M's rows number = initial state + dynamic model + equality cons +
  // inequality cons + bounding box
  int nums_of_initial_state = state_size_,
      nums_of_dynamic = state_size_ * horizon_,
      nums_of_equality_cons = A_equal_.rows() * horizon_,
      nums_of_inequality_cons = A_inequal_.rows() * horizon_,
      nums_of_state_bounding_box = x_lb_.rows() * horizon_,
      nums_of_input_bounding_box = u_lb_.rows() * horizon_;
  // pre-allocate
  M_.resize(nums_of_initial_state + nums_of_dynamic + nums_of_equality_cons +
                nums_of_inequality_cons + nums_of_state_bounding_box +
                nums_of_input_bounding_box,
            qp_state_size_);

  // initial state cons
  M_.block(0, 0, state_size_, state_size_).setIdentity();
  // dynamic
  for (size_t i = 0; i < horizon_; ++i) {
    M_.block(nums_of_initial_state + i * state_size_, i * state_size_,
             state_size_, state_size_) = Ad_;
    M_.block(nums_of_initial_state + i * state_size_, (i + 1) * state_size_,
             state_size_, state_size_) =
        -Eigen::MatrixXd::Identity(state_size_, state_size_);
    M_.block(nums_of_initial_state + i * state_size_,
             (horizon_ + i + 1) * state_size_, state_size_, input_size_) = Bd_;
  }
  // equality cons
  if (nums_of_equality_cons != 0) {
    int block_rows = A_equal_.rows();
    int start_row_offset = nums_of_initial_state + nums_of_dynamic;
    for (size_t i = 0; i < horizon_; ++i) {
      M_.block(start_row_offset + i * block_rows, i * state_size_, block_rows,
               state_size_) = A_equal_;
      M_.block(start_row_offset + i * block_rows,
               (horizon_ + 1) * state_size_ + i * input_size_, block_rows,
               input_size_) = B_equal_;
    }
  }
  // inequality cons
  if (nums_of_inequality_cons != 0) {
    int block_rows = state_size_;
    int start_row_offset = nums_of_initial_state + nums_of_dynamic +
                           nums_of_equality_cons + nums_of_inequality_cons;
    for (size_t i = 0; i < horizon_; ++i) {
      M_.block(start_row_offset + i * block_rows, i * state_size_, block_rows,
               state_size_) =
          Eigen::MatrixXd::Identity(state_size_, state_size_);
    }
  }
  // state bounding box cons
  if (nums_of_state_bounding_box != 0) {
    int block_rows = state_size_;
    int start_row_offset = nums_of_initial_state + nums_of_dynamic +
                           nums_of_equality_cons + nums_of_inequality_cons;
    for (size_t i = 0; i < horizon_; ++i) {
      M_.block(start_row_offset + i * block_rows, i * state_size_, block_rows,
               state_size_) =
          Eigen::MatrixXd::Identity(state_size_, state_size_);
    }
  }
  // input bounding box cons
  if (nums_of_input_bounding_box != 0) {
    int block_rows = input_size_;
    int start_row_offset = nums_of_initial_state + nums_of_dynamic +
                           nums_of_equality_cons + nums_of_inequality_cons +
                           nums_of_state_bounding_box;
    for (size_t i = 0; i < horizon_; ++i) {
      M_.block(start_row_offset + i * block_rows,
               (horizon_ + 1) * state_size_ + i * input_size_, block_rows,
               input_size_) =
          Eigen::MatrixXd::Identity(state_size_, state_size_);
    }
  }
}
void TrajectoryTracker::calcOsqpConstraintBound() {
  int nums_of_initial_state = state_size_,
      nums_of_dynamic = state_size_ * horizon_,
      nums_of_equality_cons = A_equal_.rows() * horizon_,
      nums_of_inequality_cons = A_inequal_.rows() * horizon_,
      nums_of_state_bounding_box = x_lb_.rows() * horizon_,
      nums_of_input_bounding_box = u_lb_.rows() * horizon_;
  int nums_of_cons_rows = nums_of_initial_state + nums_of_dynamic +
                          nums_of_equality_cons + nums_of_inequality_cons +
                          nums_of_state_bounding_box +
                          nums_of_input_bounding_box;
  lb_.resize(nums_of_cons_rows);
  ub_.resize(nums_of_cons_rows);

  // initial state cons
  lb_.segment(0, state_size_) = init_state_;
  ub_.segment(0, state_size_) = init_state_;
  // dynamic cons
  for (size_t i = 0; i < horizon_; ++i) {
    lb_.segment(nums_of_initial_state + i * state_size_, state_size_) = Kd_;
    ub_.segment(nums_of_initial_state + i * state_size_, state_size_) = Kd_;
  }
  // equality cons
  if (nums_of_equality_cons != 0) {
    int block_rows = A_equal_.rows();
    int start_row_offset = nums_of_initial_state + nums_of_dynamic;
    for (size_t i = 0; i < horizon_; ++i) {
      lb_.segment(start_row_offset + i * block_rows, block_rows) = K_equal_;
      ub_.segment(start_row_offset + i * block_rows, block_rows) = K_equal_;
    }
  }
  // inequality cons
  if (nums_of_inequality_cons != 0) {
    int block_rows = state_size_;
    int start_row_offset = nums_of_initial_state + nums_of_dynamic +
                           nums_of_equality_cons + nums_of_inequality_cons;
    for (size_t i = 0; i < horizon_; ++i) {
      lb_.segment(start_row_offset + i * block_rows, block_rows) =
          K_inequal_lb_;
      ub_.segment(start_row_offset + i * block_rows, block_rows) =
          K_inequal_ub_;
    }
  }
  // state bounding box cons
  if (nums_of_state_bounding_box != 0) {
    int block_rows = state_size_;
    int start_row_offset = nums_of_initial_state + nums_of_dynamic +
                           nums_of_equality_cons + nums_of_inequality_cons;
    for (size_t i = 0; i < horizon_; ++i) {
      lb_.segment(start_row_offset + i * block_rows, block_rows) = x_lb_;
      ub_.segment(start_row_offset + i * block_rows, block_rows) = x_ub_;
    }
  }
  // input bounding box cons
  if (nums_of_input_bounding_box != 0) {
    int block_rows = input_size_;
    int start_row_offset = nums_of_initial_state + nums_of_dynamic +
                           nums_of_equality_cons + nums_of_inequality_cons +
                           nums_of_state_bounding_box;
    for (size_t i = 0; i < horizon_; ++i) {
      lb_.segment(start_row_offset + i * block_rows, block_rows) = u_lb_;
      ub_.segment(start_row_offset + i * block_rows, block_rows) = u_ub_;
    }
  }

  cons_bd_.resize(nums_of_cons_rows, 2);
  cons_bd_.col(0) = lb_;
  cons_bd_.col(1) = ub_;
}

}; // namespace willand_ackermann
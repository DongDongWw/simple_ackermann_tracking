#pragma once
#include "Eigen/Dense"
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/src/SparseCore/SparseMatrix.h>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

#define kEps 2.22507e-308

namespace willand_ackermann {
class TrajectoryTracker {
public:
  typedef std::unique_ptr<TrajectoryTracker> UniquePtr;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DMatrix;
  typedef Eigen::SparseMatrix<double> SparseMatrix;
  typedef Eigen::VectorXd DVector;
  typedef Eigen::Vector2d Point2d;
  typedef Eigen::Vector2d Vector2d;
  typedef std::vector<Eigen::Vector2d> Trajectory2D;
  typedef std::vector<Eigen::VectorXd> TrajectoryXD;
  typedef std::function<DMatrix(double, const DVector &, const DVector &)>
      MatrixCaster;
  typedef std::function<DVector(double, const DVector &, const DVector &)>
      VectorCaster;

private:
  // parameters of mpc
  int state_size_;
  int input_size_;
  int horizon_;
  double interval_;
  DVector init_state_;
  DMatrix Q_, R_; // weight matrices
  DMatrix Ad_;
  DMatrix Bd_;
  DVector Kd_;
  DMatrix A_equal_;
  DMatrix B_equal_;
  DVector K_equal_;
  DMatrix A_inequal_;
  DMatrix B_inequal_;
  DVector K_inequal_lb_;
  DVector K_inequal_ub_;
  DVector x_lb_;
  DVector x_ub_;
  DVector u_lb_;
  DVector u_ub_;
  // parameters of qp
  int qp_state_size_;
  SparseMatrix H_;
  DVector g_;
  SparseMatrix M_; // constraint matrices
  DVector lb_, ub_;
  // DMatrix cons_bd_;
  // std::vector<DMatrix> Ad_seq_;
  // std::vector<DMatrix> Bd_seq_;
  // std::vector<DMatrix> Kd_seq_;
  const Trajectory2D *refer_traj_ptr_;
  TrajectoryXD refer_state_seq_;
  TrajectoryXD refer_input_seq_;
  MatrixCaster DynamicStateMatrixCaster;
  MatrixCaster DynamicInputMatrixCaster;
  VectorCaster DynamicVectorCaster;

private:
  void calcOsqpHession();
  void calcOsqpGradient();
  void calcOsqpConstraintMatrix();
  void calcOsqpConstraintBound();
  inline bool setInitialState(const DVector &init_state) {
    if (init_state.rows() != state_size_) {
      std::cout << "Invalid initial state" << std::endl;
      return false;
    }
    init_state_ = init_state;
    return true;
  }
  inline bool setWeightMatrices(const DMatrix &Q, const DMatrix &R) {
    if (Q.rows() != state_size_ || Q.cols() != state_size_ ||
        R.rows() != input_size_ || R.cols() != input_size_) {
      std::cout << "Invalid weight matrix" << std::endl;
      return false;
    }
    Q_ = Q;
    R_ = R;
    return true;
  }
  inline void
  setDynamicParamsCaster(const MatrixCaster &dynamic_state_matrix_caster,
                         const MatrixCaster &dynamic_input_matrix_caster,
                         const VectorCaster &dynamic_vector_caster) {
    DynamicStateMatrixCaster = dynamic_state_matrix_caster;
    DynamicInputMatrixCaster = dynamic_input_matrix_caster;
    DynamicVectorCaster = dynamic_vector_caster;
  }

  inline void setEqualityConstraints(const DMatrix &A, const DMatrix &B,
                                     const DVector &K) {
    // if (A.rows() == 0 || A.cols() == 0 || B.rows() == 0 || B.cols() == 0 ||
    //     K.rows() == 0) {
    //   std::cout << "Empty equality constraints matrix or vector" <<
    //   std::endl; return false;
    // }
    // if (A.rows() != B.rows() || A.cols() != state_size_ ||
    //     B.cols() != input_size_ || K.rows() != A.rows()) {
    //   std::cout << "Invalid size of equality constraints matrix or vector"
    //             << std::endl;
    //   return false;
    // }
    A_equal_ = A;
    B_equal_ = B;
    K_equal_ = K;
    // return true;
  }

  inline void setInequalityConstraints(const DMatrix &A, const DMatrix &B,
                                       const DVector &lb, const DVector &ub) {
    // if (A.rows() == 0 || A.cols() == 0 || B.rows() == 0 || B.cols() == 0 ||
    //     lb.rows() == 0 || ub.rows() == 0) {
    //   std::cout << "Invalid inequality constraints matrix or vector"
    //             << std::endl;
    //   return false;
    // }
    // if (A.rows() != B.rows() || A.rows() != lb.rows() ||
    //     A.rows() != ub.rows() || A.cols() != state_size_ ||
    //     B.cols() != input_size_) {
    //   std::cout << "Invalid size of inequality constraints matrix or vector"
    //             << std::endl;
    //   return false;
    // }
    A_inequal_ = A;
    B_inequal_ = B;
    K_inequal_lb_ = lb;
    K_inequal_ub_ = ub;
    // return true;
  }
  inline void setBoundBoxConstraints(const DVector &x_lb, const DVector &x_ub,
                                     const DVector &u_lb, const DVector &u_ub) {
    // if (x_lb.size() != state_size_ || x_ub.size() != state_size_ ||
    //     u_lb.size() != input_size_ || u_ub.size() != input_size_) {
    //   std::cout << "Invalid size of bounding box vector" << std::endl;
    //   return false;
    // }
    x_lb_ = x_lb;
    x_ub_ = x_ub;
    u_lb_ = u_lb;
    u_ub_ = u_ub;
    // return true;
  }

  inline bool setReferenceTrajectory(const Trajectory2D &refer_traj) {
    if (refer_traj.size() <= 1) {
      std::cout << "Invalid reference trajectory" << std::endl;
      return false;
    }

    // given a 2-d trajectory, calculate the reference states and inputs
    // calculate reference states: (x, y, theta and velocity)
    refer_state_seq_.clear();
    refer_state_seq_.reserve(horizon_ + 1);
    for (size_t i = 0; i < horizon_; ++i) {
      auto &refer_state = refer_state_seq_.at(i);
      refer_state.resize(state_size_);
      refer_state.segment(0, 2) = refer_traj.at(i);
      double delta_x = refer_traj.at(i + 1)(0) - refer_traj.at(i)(0);
      double delta_y = refer_traj.at(i + 1)(1) - refer_traj.at(i)(1);
      refer_state(2) = std::atan(delta_y / (delta_x + kEps));
      refer_state(3) =
          std::sqrt(delta_x * delta_x + delta_y * delta_y) / interval_;
    }
    auto &refer_state = refer_state_seq_.back();
    refer_state.resize(state_size_);
    refer_state.segment(0, 2) = refer_traj.back();
    refer_state.segment(2, 2) = refer_state_seq_.at(horizon_ - 2).segment(2, 2);

    // calculate approximate inputs: omega and accelaration
    refer_input_seq_.clear();
    refer_input_seq_.reserve(horizon_);
    for (size_t i = 1; i < horizon_; ++i) {
      auto &refer_input = refer_input_seq_.at(i);
      double acc = (refer_state_seq_.at(i + 1)(2) - refer_state_seq_.at(i)(2)) /
                   interval_;
      refer_input.resize(input_size_);
      refer_input(1) = acc;
      // calculate curvature by three points
      // a -> b -> c
      Point2d A = refer_state_seq_.at(i - 1).segment<2>(0);
      Point2d B = refer_state_seq_.at(i).segment<2>(0);
      Point2d C = refer_state_seq_.at(i + 1).segment<2>(0);
      Vector2d ab = B - A, ac = C - A, bc = C - B;
      double angle_included =
          std::acos(ab.dot(ac) / (ab.norm() * ac.norm() + kEps));
      double radius = bc.norm() / 2 / std::sin(angle_included);
      double omega = refer_state_seq_.at(i)(2) / radius;
      refer_input(0) = omega;
    }
    // the first input vector is not be calculated
    refer_input_seq_.front() = refer_input_seq_.at(1);
    return true;
  }

  void CastProblemToQpForm();

public:
  TrajectoryTracker(int state_size, int input_size, int horizon,
                    double interval);

  bool init(const DMatrix &Q, const DMatrix &R,
            const MatrixCaster &dynamic_state_matrix_caster,
            const MatrixCaster &dynamic_input_matrix_caster,
            const VectorCaster &dynamic_vector_caster, const DMatrix &A_equal,
            const DMatrix &B_equal, const DVector &K_equal,
            const DMatrix &A_inequal, const DMatrix &B_inequal,
            const DVector &K_inequal_lb, const DVector &K_inequal_ub,
            const DVector &x_lb, const DVector &x_ub, const DVector &u_lb,
            const DVector &u_ub, const DVector &init_state,
            const Trajectory2D &reference_traj);
  bool solve(DVector &solution);
};

} // namespace willand_ackermann
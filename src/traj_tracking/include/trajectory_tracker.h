#pragma once
#include "Eigen/Dense"
#include "OsqpEigen/OsqpEigen.h"
#include <vector>
namespace willand_ackermann {
class TrajectoryTracker {
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DMatrix;
  typedef Eigen::VectorXd DVector;
  typedef std::vector<Eigen::Vector2d> Trajectory2D;
  typedef Eigen::Vector2d Point2d;

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
  DMatrix H_;
  DMatrix g_;
  DMatrix M_; // constraint matrices
  DVector lb_, ub_;
  DMatrix cons_bd_;
  const Trajectory2D *refer_traj_ptr_;

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

  inline bool setDynamicMatrices(const DMatrix &A, const DMatrix &B,
                                 const DVector &K) {
    if (A.rows() == 0 || A.cols() == 0 || B.rows() == 0 || B.cols() == 0 ||
        K.rows() == 0) {
      std::cout << "Invalid dynamic matrix or vector" << std::endl;
      return false;
    }
    if (A.rows() != state_size_ || A.cols() != state_size_ ||
        B.rows() != state_size_ || B.cols() != input_size_ ||
        K.rows() != state_size_) {
      std::cout << "Invalid size of dynamic matrix or vector" << std::endl;
      return false;
    }
    Ad_ = A;
    Bd_ = B;
    Kd_ = K;
    return true;
  }

  inline bool setEqualityConstraints(const DMatrix &A, const DMatrix &B,
                                     const DVector &K) {
    if (A.rows() == 0 || A.cols() == 0 || B.rows() == 0 || B.cols() == 0 ||
        K.rows() == 0) {
      std::cout << "Invalid equality constraints matrix or vector" << std::endl;
      return false;
    }
    if (A.rows() != B.rows() || A.cols() != state_size_ ||
        B.cols() != input_size_ || K.rows() != A.rows()) {
      std::cout << "Invalid size of equality constraints matrix or vector"
                << std::endl;
      return false;
    }
    A_equal_ = A;
    B_equal_ = B;
    K_equal_ = K;
    return true;
  }

  inline bool setInequalityConstraints(const DMatrix &A, const DMatrix &B,
                                       const DVector &lb, const DVector &ub) {
    if (A.rows() == 0 || A.cols() == 0 || B.rows() == 0 || B.cols() == 0 ||
        lb.rows() == 0 || ub.rows() == 0) {
      std::cout << "Invalid inequality constraints matrix or vector"
                << std::endl;
      return false;
    }
    if (A.rows() != B.rows() || A.rows() != lb.rows() ||
        A.rows() != ub.rows() || A.cols() != state_size_ ||
        B.cols() != input_size_) {
      std::cout << "Invalid size of inequality constraints matrix or vector"
                << std::endl;
      return false;
    }
    A_inequal_ = A;
    B_inequal_ = B;
    K_inequal_lb_ = lb;
    K_inequal_ub_ = ub;
    return true;
  }
  inline bool setBoundBoxConstraints(const DVector &x_lb, const DVector &x_ub,
                                     const DVector &u_lb, const DVector &u_ub) {
    if (x_lb.size() != state_size_ || x_ub.size() != state_size_ ||
        u_lb.size() != input_size_ || u_ub.size() != input_size_) {
      std::cout << "Invalid size of bounding box vector" << std::endl;
      return false;
    }
    x_lb_ = x_lb;
    x_ub_ = x_ub;
    u_lb_ = u_lb;
    u_ub_ = u_ub;
    return true;
  }

  inline bool
  setReferenceTrajectory(const Trajectory2D &refer_traj) {
    if (refer_traj.size() == 0) {
      std::cout << "Invalid reference trajectory" << std::endl;
      return false;
    }
    refer_traj_ptr_ = &refer_traj;
    return true;
  }

public:
  TrajectoryTracker(int state_size, int input_size, int horizon,
                    double interval);

  bool init(const DMatrix &Q, const DMatrix &R, const DMatrix &Ad,
            const DMatrix &Bd, const DVector &Kd, const DMatrix &A_equal,
            const DMatrix &B_equal, const DVector &K_equal,
            const DMatrix &A_inequal, const DMatrix &B_inequal,
            const DVector &K_inequal_lb, const DVector &K_inequal_ub,
            const DVector &x_lb, const DVector &x_ub, const DVector &u_lb,
            const DVector &u_ub, const DVector& init_state, const Trajectory2D& reference_traj);
};

} // namespace willand_ackermann
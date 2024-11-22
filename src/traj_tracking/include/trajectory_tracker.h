#pragma once
#include "Eigen/Dense"
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/src/SparseCore/SparseMatrix.h>
#include <cmath>
#include <functional>
#include <iomanip>
#include <ios>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#define kEps 1.0e-6
namespace willand_ackermann {
struct TrackerParam {
  // mpc parameters
  int horizon_;
  double interval_;
  int state_size_;
  int input_size_;
  double speed_limit_;
  double acc_limit_;
  double front_wheel_angle_limit_;
  double front_wheel_angle_rate_limit_;
  // vehicle parameters
  double track_width_;
  double dist_front_to_rear_;

  TrackerParam()
      : horizon_(20), interval_(0.2), state_size_(4), input_size_(2),
        speed_limit_(1.0), acc_limit_(1.0), front_wheel_angle_limit_(M_PI / 4),
        front_wheel_angle_rate_limit_(M_PI / 8), track_width_(0.5),
        dist_front_to_rear_(0.8) {}
  TrackerParam(int horizon, double interval, int state_size, int input_size,
               double speed_limit, double acc_limit,
               double front_wheel_angle_limit,
               double front_wheel_angle_rate_limit, double track_width,
               double dist_front_to_rear);
};
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
  typedef std::function<DMatrix(const TrackerParam &param, const DVector &,
                                const DVector &)>
      MatrixCaster;
  typedef std::function<DVector(const TrackerParam &param, const DVector &,
                                const DVector &)>
      VectorCaster;
  typedef std::function<DMatrix(const TrackerParam &param, TrajectoryXD,
                                TrajectoryXD)>
      UserCustomizeMatrixCaster;
  typedef std::function<DMatrix(const TrackerParam &param, TrajectoryXD,
                                TrajectoryXD)>
      UserCustomizeBoundCaster;

private:
  TrackerParam param_;
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
  DMatrix A_steer_rate_;
  DVector lb_steer_rate_;
  DVector ub_steer_rate_;
  DMatrix A_accelerate_;
  DVector lb_accelerate_;
  DVector ub_accelerate_;
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
  // const Trajectory2D *refer_traj_ptr_;
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
  bool setInitialState(const DVector &init_state) {
    if (init_state.rows() != param_.state_size_) {
      std::cout << "Invalid initial state" << std::endl;
      return false;
    }
    init_state_ = init_state;
    return true;
  }
  bool setReferenceTrajectory(const Trajectory2D &refer_traj);
  void setWeightMatrices() {
    Q_.resize(param_.state_size_, param_.state_size_);
    R_.resize(param_.input_size_, param_.input_size_);
    Q_ << 100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 1.0;
    R_ << 1.0, 0.0, 0.0, 1.0;
  }
  DMatrix dynamicStateMatrixCaster(const DVector &state, const DVector &input);
  DMatrix dynamicInputMatrixCaster(const DVector &state, const DVector &input);
  DVector dynamicVectorCaster(const DVector &state, const DVector &input);
  DMatrix steerRateConstraintsMatrixCaster();
  DMatrix steerRateConstraintsBoundCaster();
  DMatrix accelerateConstraintsMatrixCaster();
  DMatrix accelerateConstraintsBoundCaster();
  void setGeneralEqualityConstraints() {}

  void setGeneralInequalityConstraints() {
    // if angle limit larger than PI/2, then the constraints are not need
    if (param_.front_wheel_angle_limit_ - M_PI / 2 > -kEps ||
        param_.front_wheel_angle_limit_ + M_PI / 2 < kEps) {
      // std::cout << "Front wheel angle limit too large, constraints abandoned"
      //           << std::endl;
      return;
    }
    A_inequal_ = DMatrix::Zero(4, param_.state_size_);
    B_inequal_ = DMatrix::Zero(4, param_.input_size_);
    B_inequal_ << -2 * std::tan(param_.front_wheel_angle_limit_),
        -(2 * param_.dist_front_to_rear_ -
          param_.track_width_ * std::tan(param_.front_wheel_angle_limit_)),
        -2 * std::tan(param_.front_wheel_angle_limit_),
        (2 * param_.dist_front_to_rear_ +
         param_.track_width_ * std::tan(param_.front_wheel_angle_limit_)),
        -2 * std::tan(param_.front_wheel_angle_limit_),
        -(2 * param_.dist_front_to_rear_ +
          param_.track_width_ * std::tan(param_.front_wheel_angle_limit_)),
        -2 * std::tan(param_.front_wheel_angle_limit_),
        (2 * param_.dist_front_to_rear_ -
         param_.track_width_ * std::tan(param_.front_wheel_angle_limit_));

    K_inequal_lb_.resize(4);
    K_inequal_lb_.setConstant(-std::numeric_limits<double>::infinity());
    K_inequal_ub_.resize(4);
    K_inequal_ub_.setZero();
  }
  void setGeneralBoundBoxConstraints() {
    x_lb_ = DVector::Zero(param_.state_size_);
    x_ub_ = DVector::Zero(param_.state_size_);
    x_lb_ << -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity();
    x_ub_ << +std::numeric_limits<double>::infinity(),
        +std::numeric_limits<double>::infinity(),
        +std::numeric_limits<double>::infinity();
    u_lb_ = DVector::Zero(param_.input_size_);
    u_ub_ = DVector::Zero(param_.input_size_);
    u_lb_ << -param_.speed_limit_, -std::numeric_limits<double>::infinity();
    u_ub_ << param_.speed_limit_, std::numeric_limits<double>::infinity();
  }
  void setSteerRateConstraints() {
    A_steer_rate_ = steerRateConstraintsMatrixCaster();
    DMatrix b = steerRateConstraintsBoundCaster();
    lb_steer_rate_ = b.col(0);
    ub_steer_rate_ = b.col(1);
  }
  void setAccelerateConstraints() {
    A_accelerate_ = accelerateConstraintsMatrixCaster();
    DMatrix b = accelerateConstraintsBoundCaster();
    lb_accelerate_ = b.col(0);
    ub_accelerate_ = b.col(1);
  }
  void CastProblemToQpForm();

public:
  TrajectoryTracker(const TrackerParam &param);

  bool update(const DVector &init_state, const Trajectory2D &reference_traj);
  bool solve(DVector &solution);
  void printRefereceStateSeq();
  void printRefereceInputSeq();
};

} // namespace willand_ackermann
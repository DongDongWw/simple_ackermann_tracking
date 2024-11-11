#include "trajectory_tracker.h"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <bits/c++config.h>
#include <cmath>
#include <limits>
#include <linux/limits.h>
#include <memory>
using namespace willand_ackermann;
#define PI 3.14159265358979323846

int main() {
  constexpr double front_wheel_angle_limit = PI / 8;
  constexpr double dist_front_to_rear = 0.8; // unit, m
  constexpr double track_width = 0.5;        // unit, m
  constexpr double speed_limit = 2.0;        // unit, m
  constexpr double acc_limit = 2.0;          // unit, m
  const double interval = 0.2;
  const int horizon = 10;   // duration = 2 secs
  const int state_size = 4; // (x, y, theta, v)
  const int input_size = 2; // (omega, acc)

  TrajectoryTracker::DMatrix Q;
  TrajectoryTracker::DMatrix R;
  TrajectoryTracker::DMatrix A_equal;
  TrajectoryTracker::DMatrix B_equal;
  TrajectoryTracker::DVector K_equal;
  TrajectoryTracker::DMatrix A_inequal;
  TrajectoryTracker::DMatrix B_inequal;
  TrajectoryTracker::DVector K_inequal_lb;
  TrajectoryTracker::DVector K_inequal_ub;
  TrajectoryTracker::DVector x_lb;
  TrajectoryTracker::DVector x_ub;
  TrajectoryTracker::DVector u_lb;
  TrajectoryTracker::DVector u_ub;
  TrajectoryTracker::DVector init_state;

  Q.resize(state_size, state_size);
  R.resize(input_size, input_size);
  Q << 100, 0, 0, 0, 100, 0, 0, 0, 100;
  R << 10, 0, 0, 10;

  A_inequal.resize(4, state_size);
  B_inequal.resize(4, input_size);
  A_inequal << 0, 0, 0, -2 * std::tan(front_wheel_angle_limit), 0, 0, 0,
      -2 * std::tan(front_wheel_angle_limit), 0, 0, 0,
      -2 * std::tan(front_wheel_angle_limit), 0, 0, 0,
      -2 * std::tan(front_wheel_angle_limit);
  B_inequal << -(2 * dist_front_to_rear -
                 track_width * std::tan(front_wheel_angle_limit)),
      0,
      (2 * dist_front_to_rear +
       track_width * std::tan(front_wheel_angle_limit)),
      0,
      -(2 * dist_front_to_rear +
        track_width * std::tan(front_wheel_angle_limit)),
      0,
      (2 * dist_front_to_rear -
       track_width * std::tan(front_wheel_angle_limit)),
      0;
  K_inequal_lb.resize(4);
  K_inequal_lb.setConstant(-std::numeric_limits<double>::infinity());
  K_inequal_ub.resize(4);
  K_inequal_ub.setZero();

  x_lb.resize(4);
  x_lb << -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(), -PI, -speed_limit;
  x_ub << +std::numeric_limits<double>::infinity(),
      +std::numeric_limits<double>::infinity(), +PI, speed_limit;
  u_lb.resize(2);
  u_lb << -std::numeric_limits<double>::infinity(), -acc_limit;
  u_ub << std::numeric_limits<double>::infinity(), acc_limit;

  init_state.resize(state_size);
  init_state << 0, 0, 0, 0;
  // funciton mapping parameters to discrete linear matrix
  auto dynamic_state_matrix_caster =
      [](double interval, const TrajectoryTracker::DVector &x_refer,
         const TrajectoryTracker::DVector &u_refer)
      -> TrajectoryTracker::DMatrix {
    int state_size = x_refer.size();
    double theta = x_refer(2), v = x_refer(3);
    TrajectoryTracker::DMatrix partial_x;
    partial_x.resize(state_size, state_size);
    partial_x << 0, 0, 0, 0, 0, 0, 0, 0, -v * std::sin(theta),
        v * std::cos(theta), 0, 0, std::cos(theta), std::sin(theta), 0, 0;
    return Eigen::MatrixXd::Identity(state_size, state_size) +
           interval * partial_x.transpose();
  };

  auto dynamic_input_matrix_caster =
      [](double interval, const TrajectoryTracker::DVector &x_refer,
         const TrajectoryTracker::DVector &u_refer)
      -> TrajectoryTracker::DMatrix {
    int input_size = u_refer.size();
    TrajectoryTracker::DMatrix partial_u;
    partial_u.resize(input_size, state_size);
    partial_u << 0, 0, 1, 0, 0, 0, 0, 1;
    return interval * partial_u.transpose();
  };

  auto dynamic_vector_caster = [](double interval,
                                  const TrajectoryTracker::DVector &x_refer,
                                  const TrajectoryTracker::DVector &u_refer)
      -> TrajectoryTracker::DMatrix {
    int state_size = x_refer.size();
    int input_size = u_refer.size();
    double theta = x_refer(2), v = x_refer(3), omege = u_refer(0),
           acc = u_refer(1);
    TrajectoryTracker::DVector x_dot(state_size);
    TrajectoryTracker::DMatrix partial_x(state_size, state_size);
    TrajectoryTracker::DMatrix partial_u(input_size, state_size);

    x_dot << v * std::cos(theta), v * std::sin(theta), omege, acc;
    partial_x << 0, 0, 0, 0, 0, 0, 0, 0, -v * std::sin(theta),
        v * std::cos(theta), 0, 0, std::cos(theta), std::sin(theta), 0, 0;
    partial_u.resize(input_size, state_size);
    partial_u << 0, 0, 1, 0, 0, 0, 0, 1;
    return interval *
           (x_dot - partial_x.transpose() * x_refer - partial_u * u_refer);
  };

  // generate reference trajectory
  TrajectoryTracker::Trajectory2D refer_traj;
  refer_traj.reserve(horizon + 1);
  double line_speed = 1.0, radius = 5.0, omega_speed = line_speed / radius;
  for (size_t i = 0; i <= horizon; ++i) {
    double angle = omega_speed * i * interval;
    double x = radius * std::cos(angle), y = radius * std::sin(angle);
    auto &refer_state = refer_traj.at(i);
    refer_state << x, y;
  }

  TrajectoryTracker::UniquePtr traj_tracker =
      std::make_unique<TrajectoryTracker>(state_size, input_size, horizon,
                                          interval);
  traj_tracker->init(Q, R, dynamic_state_matrix_caster,
                     dynamic_input_matrix_caster, dynamic_vector_caster,
                     A_equal, B_equal, K_equal, A_inequal, B_inequal,
                     K_inequal_lb, K_inequal_ub, x_lb, x_ub, u_lb, u_ub,
                     init_state, refer_traj);
  TrajectoryTracker::DVector solution;
  traj_tracker->solve(solution);
  TrajectoryTracker::DVector cur_ctrl =
      solution.segment(state_size * (horizon + 1), input_size);
  std::cout << cur_ctrl << std::endl;
}
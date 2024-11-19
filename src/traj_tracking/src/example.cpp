#include "trajectory_tracker.h"
#include <Eigen/Dense>
#include <Eigen/src/Core/IO.h>
#include <Eigen/src/Core/Matrix.h>
#include <bits/c++config.h>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <ios>
#include <iostream>
#include <limits>
#include <linux/limits.h>
#include <memory>
using namespace willand_ackermann;

int main() {
  const int horizon = 20;                                   // duration = 8 secs
  const double interval = 0.2;                              // unit, sec
  const int state_size = 3;                                 // (x, y, theta)
  const int input_size = 2;                                 // (v, omega)
  constexpr double speed_limit = 2.0;                       // unit, m / s
  constexpr double acc_limit = 2.0;                         // unit, m / s^2
  constexpr double front_wheel_angle_limit = M_PI / 2;      // unit, rad
  constexpr double front_wheel_angle_rate_limit = M_PI / 2; // unit, rad per sec
  constexpr double track_width = 0.5;                       // unit, m
  constexpr double dist_front_to_rear = 0.8;                // unit, m
  TrackerParam param(horizon, interval, state_size, input_size, speed_limit,
                     acc_limit, front_wheel_angle_limit,
                     front_wheel_angle_rate_limit, track_width,
                     dist_front_to_rear);
  // generate reference trajectory
  TrajectoryTracker::Trajectory2D refer_traj;
  refer_traj.reserve(horizon + 1);
  refer_traj.resize(horizon + 1);
  double line_speed = 1.0, radius = 5.0, omega_speed = line_speed / radius;
  for (size_t i = 0; i <= horizon; ++i) {
    double angle = omega_speed * i * interval;
    double x = radius * std::cos(angle), y = radius * std::sin(angle);
    auto &refer_state = refer_traj.at(i);
    refer_state << x, y;
  }
  // initial state
  TrajectoryTracker::DVector init_state(state_size);
  init_state << 5.0, 0.0, 0.0;
  // initialize trajectory tracker
  TrajectoryTracker::UniquePtr traj_tracker =
      std::make_unique<TrajectoryTracker>(param);
  traj_tracker->init(init_state, refer_traj);

  TrajectoryTracker::DVector solution;
  traj_tracker->solve(solution);

  Eigen::IOFormat CleanFmt(4, Eigen::DontAlignCols, ", ", "", "(", ")");
  for (size_t i = 0; i <= horizon; ++i) {
    Eigen::Vector2d x = solution.segment(i * (state_size), 2);
    double dist = (refer_traj.at(i) - x).norm();
    std::cout << std::fixed << std::setprecision(2) << std::setw(4)
              << std::showpos << "time stamp = " << i * interval
              << ", reference state = "
              << refer_traj.at(i).transpose().format(CleanFmt)
              << ", planning state = " << x.transpose().format(CleanFmt)
              << ", error = " << dist << std::endl;
  }
  traj_tracker->printRefereceStateSeq();
  traj_tracker->printRefereceInputSeq();
  // traj_tracker->printOsqpMatrices();
}
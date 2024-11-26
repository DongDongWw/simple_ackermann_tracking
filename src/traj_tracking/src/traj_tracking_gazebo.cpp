#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

#include "geometry_msgs/Twist.h"
#include "ros/node_handle.h"
#include "ros/timer.h"
#include "tracking_server.h"
#include "trajectory_tracker.h"

using namespace willand_ackermann;

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_tracking_node");
  ros::NodeHandle nh;

  constexpr int horizon = 20;                           // duration = 8 secs
  constexpr double interval = 0.02;                     // unit, sec
  constexpr int state_size = 3;                         // (x, y, theta)
  constexpr int input_size = 2;                         // (v, omega)
  constexpr double speed_limit = 1.5;                   // unit, m / s
  constexpr double acc_limit = 1.0;                     // unit, m / s^2
  constexpr double front_wheel_angle_limit = M_PI / 3;  // unit, rad
  constexpr double front_wheel_angle_rate_limit =
      M_PI / 4;                               // unit, rad per sec
  constexpr double track_width = 0.4;         // unit, m
  constexpr double dist_front_to_rear = 0.4;  // unit, m
  TrackerParam param(horizon, interval, state_size, input_size, speed_limit,
                     acc_limit, front_wheel_angle_limit,
                     front_wheel_angle_rate_limit, track_width,
                     dist_front_to_rear);
  constexpr double scale_coef = 0.8;
  double interval_between_points = scale_coef * speed_limit * interval;
  PathGenerator path_generator(interval_between_points);
  TrackingServer tracking_server(param, path_generator);
  tracking_server.init(nh);
  ros::spin();

  return 0;
}
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

#include "geometry_msgs/Twist.h"
#include "mpc_tracker.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/timer.h"
#include "tracking_server.h"

using namespace willand_ackermann;

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_tracking");
  ros::NodeHandle nh;
  int horizon, state_size, input_size;
  double interval, min_vel, max_vel, min_acc, max_acc, steer_angle_rate_limit,
      track_width, wheel_base;
  double weight_x_error, weight_y_error, weight_theta_error, weight_v,
      weight_omega;

  constexpr double ref_vel = 1.0;
  double interval_between_points = ref_vel * interval;
  TrackingServer tracking_server(nh);
  tracking_server.startSimulate();
  return 0;
}

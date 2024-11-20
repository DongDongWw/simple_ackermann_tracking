#include "QuinticCurveGenerator.h"
#include "geometry_msgs/Twist.h"
#include "ros/node_handle.h"
#include "ros/timer.h"
#include "trajectory_tracker.h"
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace willand_ackermann;

// mpc parameters
const int horizon = 40;                                   // horizon
const double interval = 0.1;                              // unit, sec
const int state_size = 3;                                 // (x, y, theta)
const int input_size = 2;                                 // (v, omega)
constexpr double speed_limit = 2.0;                       // unit, m / s
constexpr double acc_limit = 2.0;                         // unit, m / s^2
constexpr double front_wheel_angle_limit = M_PI / 2;      // unit, rad
constexpr double front_wheel_angle_rate_limit = M_PI / 4; // unit, rad per sec
constexpr double track_width = 0.4;                       // unit, m
constexpr double dist_front_to_rear = 0.4;                // unit, m

QuinticCurveGenerator curve_generator;
TrajectoryTracker::DVector initial_state = TrajectoryTracker::DVector::Zero(3);
TrajectoryTracker::Trajectory2D refer_traj;
std::vector<QuinticCurveGenerator::Point2D> global_path_points;
std::vector<QuinticCurveGenerator::Point2D> local_path_points;
TrackerParam param;

double inner_radius = 5.0;
double outer_radius = 10.0;
double refer_speed = 0.5;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

geometry_msgs::Twist cmd;

QuinticCurveGenerator::Point2D randomPointInAnnulus(double inner_radius,
                                                    double outer_radius) {
  std::random_device rd;
  std::mt19937 gen(rd());

  std::uniform_real_distribution<> radius_dist(inner_radius, outer_radius);
  double r = radius_dist(gen);

  std::uniform_real_distribution<> angle_dist(initial_state(2) - M_PI / 4,
                                              initial_state(2) + M_PI);
  double theta = angle_dist(gen);

  QuinticCurveGenerator::Point2D p;
  p(0) = r * std::cos(theta);
  p(1) = r * std::sin(theta);

  return p;
}

void publishGlobalTrajectory(const ros::TimerEvent &) {
  ros::NodeHandle nh;
  ros::Publisher global_traj_pub =
      nh.advertise<visualization_msgs::Marker>("global_traj", 10);
  global_path_points = curve_generator.getGlobalPath(
      initial_state.segment(0, 2),
      initial_state.segment(0, 2) +
          randomPointInAnnulus(inner_radius, outer_radius),
      interval * refer_speed);

  visualization_msgs::Marker global_traj;
  global_traj.header.frame_id = "odom";
  global_traj.header.stamp = ros::Time::now();
  global_traj.ns = "vehicle_traj";
  global_traj.id = 2;
  global_traj.type = visualization_msgs::Marker::LINE_STRIP;
  global_traj.action = visualization_msgs::Marker::ADD;
  global_traj.scale.x = 0.05;
  global_traj.color.r = 1.0;
  global_traj.color.g = 0.0;
  global_traj.color.b = 0.0;
  global_traj.color.a = 1.0;
  global_traj.pose.orientation.w = 1.0;
  for (auto &p : global_path_points) {
    geometry_msgs::Point cur;
    cur.x = p(0);
    cur.y = p(1);
    global_traj.points.push_back(cur);
  }
  global_traj_pub.publish(global_traj);
};

void publishLocalTrajectory(const ros::TimerEvent &) {
  local_path_points = curve_generator.generateReferenceTrajectory(
      initial_state.segment(0, 2), param.horizon_ + 1);

  ros::NodeHandle nh;
  ros::Publisher local_traj_pub =
      nh.advertise<visualization_msgs::Marker>("local_traj", 10);

  visualization_msgs::Marker local_traj;
  local_traj.header.frame_id = "odom";
  local_traj.header.stamp = ros::Time::now();
  local_traj.ns = "vehicle_traj";
  local_traj.id = 2;
  local_traj.type = visualization_msgs::Marker::LINE_STRIP;
  local_traj.action = visualization_msgs::Marker::ADD;
  local_traj.scale.x = 0.05;
  local_traj.color.r = 0.0;
  local_traj.color.g = 0.0;
  local_traj.color.b = 1.0;
  local_traj.color.a = 1.0;
  local_traj.pose.orientation.w = 1.0;
  local_traj_pub.publish(local_traj);
  std::cout << "local path published." << std::endl;
};
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  Eigen::Quaterniond quaternion(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  double yaw = std::atan2(
      2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y()),
      1.0 - 2.0 * (quaternion.y() * quaternion.y() +
                   quaternion.z() * quaternion.z()));
  initial_state << msg->pose.pose.position.x, msg->pose.pose.position.y, yaw;
}

void publishControlCommand(const ros::TimerEvent &) {
  static ros::NodeHandle nh;
  static ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>(
      "/steer_bot/ackermann_steering_controller/cmd_vel", 10);
  TrajectoryTracker::UniquePtr traj_tracker =
      std::make_unique<TrajectoryTracker>(param);
  TrajectoryTracker::DVector solution;
  traj_tracker->init(initial_state, local_path_points);
  if (traj_tracker->solve(solution)) {
    cmd.linear.x = solution(param.state_size_ * (param.horizon_ + 1));
    cmd.linear.y = 0;
    cmd.linear.z = 0;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = solution(param.state_size_ * (param.horizon_ + 1) + 1);
  } else {
    cmd.linear.x = 0;
    cmd.linear.y = 0;
    cmd.linear.z = 0;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = 0;
  }
  twist_pub.publish(cmd);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_tracking_node");
  ros::NodeHandle nh;
  // subscribe to the odometry topic
  ros::Subscriber odom_sub = nh.subscribe(
      "/steer_bot/ackermann_steering_controller/odom", 50, odomCallback);

  // publish the global trajectory, using timer
  ros::Timer timer_global_traj_pub =
      nh.createTimer(ros::Duration(20), publishGlobalTrajectory);
  // publish the local trajectory, using timer
  ros::Timer timer_local_traj_pub =
      nh.createTimer(ros::Duration(1.0), publishLocalTrajectory);
  // publish the control input to the vehicle
  ros::Timer timer_contol_cmd_pub =
      nh.createTimer(ros::Duration(interval), publishControlCommand);

  param = TrackerParam(horizon, interval, state_size, input_size, speed_limit,
                       acc_limit, front_wheel_angle_limit,
                       front_wheel_angle_rate_limit, track_width,
                       dist_front_to_rear);
  initial_state.setZero();

  // publish global reference trajectory once
  ros::Publisher global_traj_pub =
      nh.advertise<visualization_msgs::Marker>("global_traj", 20);
  global_path_points = curve_generator.getGlobalPath(
      initial_state.segment(0, 2),
      initial_state.segment(0, 2) +
          randomPointInAnnulus(inner_radius, outer_radius),
      interval * refer_speed);

  visualization_msgs::Marker global_traj;
  global_traj.header.frame_id = "odom";
  global_traj.header.stamp = ros::Time::now();
  global_traj.ns = "vehicle_traj";
  global_traj.id = 0;
  global_traj.type = visualization_msgs::Marker::LINE_STRIP;
  global_traj.action = visualization_msgs::Marker::ADD;
  global_traj.scale.x = 0.05;
  global_traj.color.r = 1.0;
  global_traj.color.g = 0.0;
  global_traj.color.b = 0.0;
  global_traj.color.a = 1.0;
  global_traj.pose.orientation.w = 1.0;
  for (auto &p : global_path_points) {
    geometry_msgs::Point cur;
    cur.x = p(0);
    cur.y = p(1);
    global_traj.points.push_back(cur);
  }
  global_traj_pub.publish(global_traj);

  ros::spin();
  return 0;
}
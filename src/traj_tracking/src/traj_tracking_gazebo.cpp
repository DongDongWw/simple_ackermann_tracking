#include "PathGenerator.h"
#include "geometry_msgs/Twist.h"
#include "ros/node_handle.h"
#include "ros/timer.h"
#include "trajectory_tracker.h"
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace willand_ackermann;

// mpc parameters
constexpr int horizon = 40;                               // horizon
constexpr double interval = 0.02;                         // unit, sec
constexpr int state_size = 3;                             // (x, y, theta)
constexpr int input_size = 2;                             // (v, omega)
constexpr double speed_limit = 2.0;                       // unit, m / s
constexpr double acc_limit = 2.0;                         // unit, m / s^2
constexpr double front_wheel_angle_limit = M_PI / 2;      // unit, rad
constexpr double front_wheel_angle_rate_limit = M_PI / 4; // unit, rad per sec
constexpr double track_width = 0.4;                       // unit, m
constexpr double dist_front_to_rear = 0.4;                // unit, m
TrajectoryTracker::DVector initial_state = TrajectoryTracker::DVector::Zero(3);
TrajectoryTracker::Trajectory2D refer_traj;
std::vector<PathGenerator::Point2D> global_path_points;
std::vector<PathGenerator::Point2D> local_path_points;
TrackerParam param;

constexpr double inner_radius = 5.0;
constexpr double outer_radius = 10.0;
constexpr double refer_inteval = 0.5 * interval;
PathGenerator curve_generator(refer_inteval);

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

geometry_msgs::Twist cmd;

PathGenerator::Point2D randomPointInAnnulus(double inner_radius,
                                            double outer_radius) {
  std::random_device rd;
  std::mt19937 gen(rd());

  std::uniform_real_distribution<> radius_dist(inner_radius, outer_radius);
  double r = radius_dist(gen);

  std::uniform_real_distribution<> angle_dist(initial_state(2) - M_PI / 3,
                                              initial_state(2) + M_PI / 3);
  double theta = angle_dist(gen);

  PathGenerator::Point2D p;
  p(0) = r * std::cos(theta);
  p(1) = r * std::sin(theta);

  return p;
}

void publishGlobalTrajectory(const ros::TimerEvent &) {
  static ros::NodeHandle nh;
  static ros::Publisher global_traj_pub =
      nh.advertise<visualization_msgs::Marker>("global_traj", 10);
  global_path_points = curve_generator.getGlobalPath(
      initial_state.segment(0, 2),
      initial_state.segment(0, 2) +
          randomPointInAnnulus(inner_radius, outer_radius),
      initial_state(2));

  visualization_msgs::Marker global_traj;
  global_traj.header.frame_id = "odom";
  global_traj.header.stamp = ros::Time::now();
  global_traj.ns = "vehicle_traj";
  global_traj.id = 1;
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
  return;
};

void publishLocalTrajectory(const ros::TimerEvent &) {
  local_path_points = curve_generator.generateReferenceTrajectory(
      initial_state.segment(0, 2), param.horizon_ + 1);
  if (local_path_points.empty()) {
    ROS_INFO("local trajectory is empty, wait ...");
    return;
  } else {
    ROS_INFO("length of local trajectory = %d",
             static_cast<int>(local_path_points.size()));
  }
  static ros::NodeHandle nh;
  static ros::Publisher local_traj_pub =
      nh.advertise<visualization_msgs::Marker>("local_traj", 10);

  visualization_msgs::Marker local_traj;
  local_traj.header.frame_id = "odom";
  local_traj.header.stamp = ros::Time::now();
  local_traj.ns = "vehicle_traj";
  local_traj.id = 2;
  local_traj.type = visualization_msgs::Marker::LINE_STRIP;
  local_traj.action = visualization_msgs::Marker::ADD;
  local_traj.scale.x = 0.1;
  local_traj.color.r = 0.0;
  local_traj.color.g = 0.0;
  local_traj.color.b = 1.0;
  local_traj.color.a = 1.0;
  local_traj.pose.orientation.w = 1.0;
  for (auto &p : local_path_points) {
    geometry_msgs::Point cur;
    cur.x = p(0);
    cur.y = p(1);
    local_traj.points.push_back(cur);
  }
  local_traj_pub.publish(local_traj);
  ROS_INFO("local path published, which length = %d",
           static_cast<int>(local_path_points.size()));
  return;
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
  // std::cout << "current state = " << initial_state.transpose() << std::endl;
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
    ROS_ERROR("Failed to solve the optimization problem");
  }
  twist_pub.publish(cmd);
}

void targetPointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  static ros::NodeHandle nh;
  static ros::Publisher global_traj_pub =
      nh.advertise<visualization_msgs::Marker>("global_traj", 10);
  PathGenerator::Point2D target_point;
  target_point << msg->pose.position.x, msg->pose.position.y;
  global_path_points = curve_generator.getGlobalPath(
      initial_state.segment(0, 2), target_point, initial_state(2));
  visualization_msgs::Marker global_traj;
  global_traj.header.frame_id = "odom";
  global_traj.header.stamp = ros::Time::now();
  global_traj.ns = "vehicle_traj";
  global_traj.id = 1;
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
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_tracking_node");
  ros::NodeHandle nh;
  // subscribe to the odometry topic
  ros::Subscriber odom_sub = nh.subscribe(
      "/steer_bot/ackermann_steering_controller/odom", 50, odomCallback);
  // subscribe to the target point topic
  ros::Subscriber sub =
      nh.subscribe("/move_base_simple/goal", 10, targetPointCallback);

  // publish the global trajectory, using timer
  ros::Timer timer_global_traj_pub =
      nh.createTimer(ros::Duration(20), publishGlobalTrajectory, false);
  // publish the local trajectory, using timer
  ros::Timer timer_local_traj_pub =
      nh.createTimer(ros::Duration(0.10), publishLocalTrajectory, false);
  // publish the control input to the vehicle
  ros::Timer timer_contol_cmd_pub =
      nh.createTimer(ros::Duration(interval), publishControlCommand);

  param = TrackerParam(horizon, interval, state_size, input_size, speed_limit,
                       acc_limit, front_wheel_angle_limit,
                       front_wheel_angle_rate_limit, track_width,
                       dist_front_to_rear);
  initial_state.setZero();

  // // publish global reference trajectory once
  // ros::Publisher global_traj_pub =
  //     nh.advertise<visualization_msgs::Marker>("global_traj", 20);
  // global_path_points = curve_generator.getGlobalPath(
  //     initial_state.segment(0, 2),
  //     initial_state.segment(0, 2) +
  //         randomPointInAnnulus(inner_radius, outer_radius));

  // visualization_msgs::Marker global_traj;
  // global_traj.header.frame_id = "odom";
  // global_traj.header.stamp = ros::Time::now();
  // global_traj.ns = "vehicle_traj";
  // global_traj.id = 0;
  // global_traj.type = visualization_msgs::Marker::LINE_STRIP;
  // global_traj.action = visualization_msgs::Marker::ADD;
  // global_traj.scale.x = 0.05;
  // global_traj.color.r = 1.0;
  // global_traj.color.g = 0.0;
  // global_traj.color.b = 0.0;
  // global_traj.color.a = 1.0;
  // global_traj.pose.orientation.w = 1.0;
  // for (auto &p : global_path_points) {
  //   geometry_msgs::Point cur;
  //   cur.x = p(0);
  //   cur.y = p(1);
  //   global_traj.points.push_back(cur);
  // }
  // global_traj_pub.publish(global_traj);

  ros::spin();
  return 0;
}
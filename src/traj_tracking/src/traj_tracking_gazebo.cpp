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

QuinticCurveGenerator curve_generator(QuinticCurveGenerator::Point2D(0.0, 0.0),
                                      QuinticCurveGenerator::Point2D(10.0,
                                                                     10.0));
TrajectoryTracker::DVector initial_state(3);
TrackerParam param;
geometry_msgs::Twist cmd;

void publishGlobalTrajectory(const ros::TimerEvent &) {
  static ros::NodeHandle nh;
  static ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("global_traj", 10);

  double refer_speed = 1.0, time_interval = 0.2;
  std::vector<QuinticCurveGenerator::Point2D> points =
      curve_generator.generateGlobalPath(time_interval * refer_speed);

  visualization_msgs::Marker global_traj;
  global_traj.header.frame_id = "odom";
  global_traj.header.stamp = ros::Time::now();
  global_traj.ns = "curve";
  global_traj.id = 0;
  global_traj.type = visualization_msgs::Marker::LINE_STRIP;
  global_traj.action = visualization_msgs::Marker::ADD;
  global_traj.scale.x = 0.05;
  global_traj.color.r = 1.0;
  global_traj.color.g = 0.0;
  global_traj.color.b = 0.0;
  global_traj.color.a = 1.0;
  global_traj.pose.orientation.w = 1.0;

  for (auto &p : points) {
    geometry_msgs::Point cur;
    cur.x = p(0);
    cur.y = p(1);
    global_traj.points.push_back(cur);
  }
  marker_pub.publish(global_traj);
};

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  static ros::NodeHandle nh;
  Eigen::Quaterniond quaternion(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  double yaw = std::atan2(
      2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y()),
      1.0 - 2.0 * (quaternion.y() * quaternion.y() +
                   quaternion.z() * quaternion.z()));

  TrajectoryTracker::DVector initial_state(3);
  initial_state << msg->pose.pose.position.x, msg->pose.pose.position.y, yaw;
  ROS_INFO("Curent vehicle positon = (%f, %f), yaw = %f", initial_state(0),
           initial_state(1), initial_state(2));
  TrajectoryTracker::Trajectory2D refer_traj =
      curve_generator.generateReferenceTrajectory(initial_state.segment(0, 2));
  TrajectoryTracker::UniquePtr traj_tracker =
      std::make_unique<TrajectoryTracker>(param);
  TrajectoryTracker::DVector solution;

  traj_tracker->init(initial_state, refer_traj);
  if (traj_tracker->solve(solution)) {
    cmd.linear.x = solution(param.state_size_ * (param.horizon_ + 1));
    cmd.linear.y = 0;
    cmd.linear.z = 0;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = solution(param.state_size_ * (param.horizon_ + 1) + 1);

    ROS_INFO("Control command solved successfully, (vel, omega) = (%f, %f)",
             cmd.linear.x, cmd.angular.z);
  }
}

void publishControlCommand(const ros::TimerEvent &) {
  static ros::NodeHandle nh;
  static ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>(
      "/steer_bot/ackermann_steering_controller/cmd_vel", 10);
  twist_pub.publish(cmd);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_tracking_node");
  ros::NodeHandle nh;
  param = TrackerParam(20, 0.2, 3, 2, 10.0, 3.0, M_PI / 2, M_PI / 3, 0.4, 0.4);

  // publish global reference trajectory
  ros::Timer timer_global_traj_pub =
      nh.createTimer(ros::Duration(1), publishGlobalTrajectory);

  // subscribe to the current state of the vehicle
  // perform trajectory tracking using MPC if the current state is received
  ros::Subscriber odom_sub = nh.subscribe(
      "/steer_bot/ackermann_steering_controller/odom", 50, odomCallback);

  // publish the control input to the vehicle
  ros::Timer timer_contol_cmd_pub =
      nh.createTimer(ros::Duration(0.2), publishControlCommand);
  ros::spin();
  return 0;
}
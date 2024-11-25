#include <string>

#include "fstream"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "path_generator.h"
#include "proto/tracking_data.pb.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "ros/timer.h"
#include "trajectory_tracker.h"
#include "visualization_msgs/Marker.h"
namespace willand_ackermann {

class TrackingServer {
 public:
  TrackingServer(const TrackerParam &mpc_param,
                 const PathGenerator &curve_generator)
      : mpc_param_(mpc_param),
        tracker_(TrajectoryTracker(mpc_param)),
        curve_generator_(curve_generator) {
    max_length_record_ = mpc_param.horizon_ * 2;
  };

  void init(ros::NodeHandle &nh) {
    odom_sub_ = nh.subscribe("/steer_bot/ackermann_steering_controller/odom",
                             50, &TrackingServer::odomCallback, this);
    nav_point_sub_ = nh.subscribe("/move_base_simple/goal", 10,
                                  &TrackingServer::targetPointCallback, this);

    timer_local_traj_pub_ =
        nh.createTimer(ros::Duration(local_traj_pub_interval),
                       &TrackingServer::publishLocalTrajectory, this);
    timer_contol_cmd_pub_ =
        nh.createTimer(ros::Duration(mpc_param_.interval_),
                       &TrackingServer::publishControlCommand, this);

    global_traj_pub_ =
        nh.advertise<visualization_msgs::Marker>("global_traj", 10);
    local_traj_pub_ =
        nh.advertise<visualization_msgs::Marker>("local_traj", 10);
    twist_cmd_pub_ = nh.advertise<geometry_msgs::Twist>(
        "/steer_bot/ackermann_steering_controller/cmd_vel", 10);
    return;
  }

 private:
  const std::string file_path_ = "../pb_data/";

  const double global_traj_pub_interval = 20.0;
  const double local_traj_pub_interval = 0.10;
  ros::Subscriber odom_sub_;
  ros::Subscriber nav_point_sub_;
  ros::Publisher global_traj_pub_;
  ros::Publisher local_traj_pub_;
  ros::Publisher twist_cmd_pub_;
  ros::Timer timer_global_traj_pub_;
  ros::Timer timer_local_traj_pub_;
  ros::Timer timer_contol_cmd_pub_;

  TrackerParam mpc_param_;
  TrajectoryTracker tracker_;
  PathGenerator curve_generator_;
  Eigen::Vector3d current_state_;
  Eigen::Vector2d current_twist_;
  Eigen::Vector2d target_point_;
  std::vector<PathGenerator::Point2D> global_path_points_;
  std::vector<PathGenerator::Point2D> local_path_points_;

  int max_length_record_;  // cann't record unlimited data
  willand_ackermann_proto::TrackingData tracking_data_;

 private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    Eigen::Quaterniond quaternion(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    double yaw = std::atan2(2.0 * (quaternion.w() * quaternion.z() +
                                   quaternion.x() * quaternion.y()),
                            1.0 - 2.0 * (quaternion.y() * quaternion.y() +
                                         quaternion.z() * quaternion.z()));
    current_state_ << msg->pose.pose.position.x, msg->pose.pose.position.y, yaw;
    current_twist_ << msg->twist.twist.linear.x, msg->twist.twist.angular.z;
  }
  void targetPointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    target_point_ << msg->pose.position.x, msg->pose.position.y;
    global_path_points_ = curve_generator_.getGlobalPath(
        current_state_.segment(0, 2), target_point_, current_state_(2));
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
    for (auto &p : global_path_points_) {
      geometry_msgs::Point cur;
      cur.x = p(0);
      cur.y = p(1);
      global_traj.points.push_back(cur);
    }
    global_traj_pub_.publish(global_traj);
  }
  void publishGlobalTrajectory(const ros::TimerEvent &) {
    auto randomPointInAnnulus = [&](double inner_radius, double outer_radius) {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> radius_dist(inner_radius, outer_radius);
      double r = radius_dist(gen);
      double theta_d = M_PI / 3;
      std::uniform_real_distribution<> angle_dist(current_state_(2) - theta_d,
                                                  current_state_(2) + theta_d);
      double theta = angle_dist(gen);

      PathGenerator::Point2D p;
      p(0) = r * std::cos(theta);
      p(1) = r * std::sin(theta);
      return p;
    };
    const double inner_radius = 5, outer_radius = 10;
    target_point_ = current_state_.segment(0, 2) +
                    randomPointInAnnulus(inner_radius, outer_radius);
    global_path_points_ = curve_generator_.getGlobalPath(
        current_state_.segment(0, 2), target_point_, current_state_(2));

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
    for (auto &p : global_path_points_) {
      geometry_msgs::Point cur;
      cur.x = p(0);
      cur.y = p(1);
      global_traj.points.push_back(cur);
    }
    global_traj_pub_.publish(global_traj);
    return;
  }
  void publishLocalTrajectory(const ros::TimerEvent &) {
    local_path_points_ = curve_generator_.generateReferenceTrajectory(
        current_state_.segment(0, 2), mpc_param_.horizon_ + 1);
    if (local_path_points_.empty()) {
      ROS_INFO("local trajectory is empty, wait ...");
      return;
    } else {
      ROS_INFO("length of local trajectory = %d",
               static_cast<int>(local_path_points_.size()));
    }
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
    for (auto &p : local_path_points_) {
      geometry_msgs::Point cur;
      cur.x = p(0);
      cur.y = p(1);
      local_traj.points.push_back(cur);
    }
    local_traj_pub_.publish(local_traj);
    ROS_INFO("local path published, which length = %d",
             static_cast<int>(local_path_points_.size()));
    return;
  }
  void publishControlCommand(const ros::TimerEvent &) {
    // stop sending control command if the vehicle is near the target point
    constexpr double threshold_near_target = 0.1;   // unit, m
    constexpr double threshold_far_off_path = 0.5;  // unit, m
    double dist_to_target =
        (current_state_.segment(0, 2) - target_point_).norm();
    double dist_off_path =
        curve_generator_.getDistOffset(current_state_.segment(0, 2));
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0;
    cmd.linear.y = 0;
    cmd.linear.z = 0;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = 0;
    if (dist_to_target < threshold_near_target ||
        dist_off_path > threshold_far_off_path || local_path_points_.empty()) {
      ROS_INFO("Control command is not sent ...");
      twist_cmd_pub_.publish(cmd);
      return;
    }
    TrajectoryTracker::DVector solution;
    tracker_.update(current_state_, local_path_points_);
    bool solve_succeed = tracker_.solve(solution);
    if (solve_succeed) {
      cmd.linear.x =
          solution(mpc_param_.state_size_ * (mpc_param_.horizon_ + 1));
      cmd.linear.y = 0;
      cmd.linear.z = 0;
      cmd.angular.x = 0;
      cmd.angular.y = 0;
      cmd.angular.z =
          solution(mpc_param_.state_size_ * (mpc_param_.horizon_ + 1) + 1);
    } else {
      ROS_ERROR("Failed to solve the optimization problem");
    }
    twist_cmd_pub_.publish(cmd);

    // save tracking data
    if (solve_succeed) {
      ros::Time cur_time = ros::Time::now();
      std::stringstream ss;
      ss << cur_time.sec << "_" << cur_time.nsec;
      tracking_data_.set_length(tracking_data_.length() + 1);
      tracking_data_.add_timestamp(ss.str());
      auto refer_data_ptr = tracking_data_.add_reference_data();
      auto actual_data_ptr = tracking_data_.add_actual_data();
      auto control_signal_ptr = tracking_data_.add_control_signal();
      // referene state
      Eigen::Vector3d refer_state;
      Eigen::Vector2d refer_input;
      tracker_.getCurrentReferStateAndSeq(refer_state, refer_input);
      // refer traj data
      refer_data_ptr->set_x(refer_state(0));
      refer_data_ptr->set_y(refer_state(1));
      refer_data_ptr->set_theta(refer_state(2));
      refer_data_ptr->set_v(refer_input(0));
      refer_data_ptr->set_omega(refer_input(1));
      refer_data_ptr->set_kappa(refer_input(1) / refer_input(0));
      // actually data
      actual_data_ptr->set_x(current_state_(0));
      actual_data_ptr->set_y(current_state_(1));
      actual_data_ptr->set_theta(current_state_(2));
      actual_data_ptr->set_v(current_twist_(0));
      actual_data_ptr->set_omega(current_twist_(1));
      actual_data_ptr->set_kappa(current_twist_(1) / current_twist_(0));
      // control data
      double v = cmd.linear.x, omega = cmd.angular.z;
      control_signal_ptr->set_v(v);
      control_signal_ptr->set_omega(omega);
      control_signal_ptr->set_kappa(omega / v);
    }
    if (tracking_data_.length() == max_length_record_) {
      tracking_data_.Clear();
    }
  }
  void serialize() {
    std::string time_stamp;
    if (tracking_data_.timestamp().empty()) {
      return;
    } else {
      time_stamp = tracking_data_.timestamp(0);
    }
    std::string file_name = file_path_ + "traj_data_" + time_stamp;
    std::ofstream output_file(file_name, std::ios::out | std::ios::binary);
    if (!tracking_data_.SerializeToOstream(&output_file)) {
      ROS_ERROR("Failed to write tracking data to file.");
    }
  }
};

};  // namespace willand_ackermann
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/PoseArray.h"
#include "mpc_tracker.h"
#include "nav_msgs/Odometry.h"
#include "ros/duration.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include "ros/timer.h"
#include "tracking_data.pb.h"
#include "visualization_msgs/Marker.h"
namespace willand_ackermann {

class TrackingServer {
 public:
  TrackingServer(const ros::NodeHandle &nh) : nh_(nh) {
    int horizon, state_size, input_size;
    double interval, min_vel, max_vel, min_acc, max_acc, steer_angle_rate_limit,
        track_width, wheel_base;
    double weight_x_error, weight_y_error, weight_theta_error, weight_v,
        weight_omega;
    nh_.param("/traj_tracking/horizon", horizon, 20);
    nh_.param("/traj_tracking/interval", interval, 0.05);
    nh_.param("/traj_tracking/state_size", state_size, 3);
    nh_.param("/traj_tracking/input_size", input_size, 2);
    nh_.param("/traj_tracking/min_vel", min_vel, -1.5);
    nh_.param("/traj_tracking/max_vel", max_vel, 1.5);
    nh_.param("/traj_tracking/steer_angle_rate_limit", steer_angle_rate_limit,
              M_PI * 2);
    nh_.param("/traj_tracking/track_width", track_width, 0.6);
    nh_.param("/traj_tracking/wheel_base", wheel_base, 1.0);
    nh_.param("/traj_tracking/weight_x_error", weight_x_error, 3333.3);
    nh_.param("/traj_tracking/weight_y_error", weight_y_error, 3333.3);
    nh_.param("/traj_tracking/weight_theta_error", weight_theta_error, 333.3);
    nh_.param("/traj_tracking/weight_v", weight_v, 63.3);
    nh_.param("/traj_tracking/weight_omega", weight_omega, 63.3);

    nh_.param("/traj_tracking/simulate_time", simulate_time_, 20.0);
    nh_.param("/traj_tracking/refer_speed", refer_speed_, 20.0);

    TrackerParam param(horizon, interval, state_size, input_size,
                       weight_x_error, weight_y_error, weight_theta_error,
                       weight_v, weight_omega, max_vel, min_vel, max_acc,
                       min_acc, steer_angle_rate_limit, track_width,
                       wheel_base);
    tracker_.reset(new MpcTracker(param));
    ROS_INFO(
        "horizon: %d,  interval: %f, state_size: %d, input_size: %d, "
        "min_vel: %f, max_vel: %f, steer_angle_rate_limit: %f, track_width: "
        "%f, wheel_base: %f, weight_x_error: %f, weight_y_error: %f, "
        "weight_theta_error: %f, weight_v: %f, weight_omega: %f",
        horizon, interval, state_size, input_size, min_vel, max_vel,
        steer_angle_rate_limit, track_width, wheel_base, weight_x_error,
        weight_y_error, weight_theta_error, weight_v, weight_omega);

    // clear tracking data directory
    const char *directory_path = "/tmp/ros/proto/traj_tracking/";
    std::string rm_cmd =
        "rm -rf " + std::string(directory_path);  // Linux/macOS命令
    int result = system(rm_cmd.c_str());
    if (result == 0) {
      ROS_INFO("Old tracking data directory removed");
    }
    std::string mkdir_cmd = "mkdir -p " + std::string(directory_path);
    result = system(mkdir_cmd.c_str());
    if (result == 0) {
      ROS_INFO("New tracking data directory created");
    }

    // subscribe the coverage planning result, by ethz
    global_path_sub_ = nh_.subscribe(
        "/waypoint_list", 1, &TrackingServer::globalPathCallBack, this);
  };

  void startSimulate() {
    while (!receive_global_path_) {
      ros::Duration(0.5).sleep();
      ros::spinOnce();
      ROS_INFO("Waiting for global path...");
    }
    int sim_steps = simulate_time_ / mpc_param_.interval_;
    int cur_step = 0;

    while (cur_step < sim_steps) {
      double ts = mpc_param_.interval_ * cur_step;

      auto refer_state_ptr = tracking_data_.add_refer_state();
      auto actual_state_ptr = tracking_data_.add_actual_state();
      auto ctrl_cmd_ptr = tracking_data_.add_ctrl_cmd();

      // calculate the local reference path
      calcLocalReferencePath(current_state_.segment(0, 2));
      tracker_->update(current_state_, local_traj_);
      Eigen::Vector2d control_cmd;
      tracker_->solve(control_cmd);

      // record simulate dates
      current_twist_ = control_cmd;
      tracking_data_.add_timestamp(ts);
      Eigen::Vector3d refer_state;
      Eigen::Vector2d refer_input;
      tracker_->getCurrentReferStateAndInput(refer_state, refer_input);

      refer_state_ptr->set_x(refer_state[0]);
      refer_state_ptr->set_y(refer_state[1]);
      refer_state_ptr->set_theta(refer_state[2]);

      actual_state_ptr->set_x(current_state_[0]);
      actual_state_ptr->set_y(current_state_[1]);
      actual_state_ptr->set_theta(current_state_[2]);

      ctrl_cmd_ptr->set_v(control_cmd[0]);
      ctrl_cmd_ptr->set_omega(control_cmd[1]);

      current_state_ = getNextState(current_state_, current_twist_);
      ++cur_step;
    }

    auto mpc_param__ptr = tracking_data_.mutable_mpc_param();
    mpc_param__ptr->set_horizon(mpc_param_.horizon_);
    mpc_param__ptr->set_interval(mpc_param_.interval_);
    mpc_param__ptr->set_state_dim(mpc_param_.state_size_);
    mpc_param__ptr->set_input_dim(mpc_param_.input_size_);
    mpc_param__ptr->set_min_vel(mpc_param_.min_vel_);
    mpc_param__ptr->set_max_vel(mpc_param_.max_vel_);
    mpc_param__ptr->set_min_acc(mpc_param_.min_acc_);
    mpc_param__ptr->set_max_acc(mpc_param_.max_acc_);
    mpc_param__ptr->set_steer_angle_rate_limit(
        mpc_param_.steer_angle_rate_limit_);
    mpc_param__ptr->set_track_width(mpc_param_.track_width_);
    mpc_param__ptr->set_wheel_base(mpc_param_.wheel_base_);

    serialize();
    return;
  }

 private:
  double simulate_time_;
  double refer_speed_;

  ros::NodeHandle nh_;
  ros::Subscriber global_path_sub_;

  const std::string file_path_ = "/tmp/ros/proto/traj_tracking/";
  TrackerParam mpc_param_;
  std::unique_ptr<MpcTracker> tracker_;
  Eigen::Vector3d current_state_;
  Eigen::Vector2d current_twist_;

  std::vector<Eigen::Vector2d> global_traj_;
  std::vector<Eigen::Vector2d> local_traj_;

  bool receive_global_path_ = false;

  // for protobuf visualization
  willand_ackermann_proto::TrackingData tracking_data_;

 private:
  void globalPathCallBack(const geometry_msgs::PoseArray::ConstPtr &msg) {
    receive_global_path_ = true;

    global_traj_.clear();
    for (auto &p : msg->poses) {
      Eigen::Vector2d point;
      point << p.position.x, p.position.y;
      global_traj_.push_back(point);
    }
    if (global_traj_.size() <= 2) {
      ROS_ERROR("Global path is empty or too short !");
      return;
    }
    double yaw = std::atan2(global_traj_.at(1)[1] - global_traj_.at(0)[1],
                            global_traj_.at(1)[0] - global_traj_.at(0)[0]);
    current_state_ =
        Eigen::Vector3d(global_traj_.at(0)[0], global_traj_.at(0)[1], yaw);
    ROS_INFO("Global path is received, and the length = %lu",
             global_traj_.size());
    return;
  }

  void calcLocalReferencePath(const Eigen::Vector2d &current_pos) {
    int p_num = mpc_param_.horizon_ + 1;
    if (global_traj_.empty()) {
      ROS_ERROR("Global path is empty !");
      return;
    }
    double min_dist = std::numeric_limits<double>::max();
    int min_dist_idx = 0;
    for (size_t i = 0; i < global_traj_.size(); ++i) {
      double dist = (global_traj_.at(i) - current_pos).norm();
      if (dist < min_dist) {
        min_dist = dist;
        min_dist_idx = i;
      }
    }
    local_traj_.clear();
    int cur_idx = min_dist_idx;
    double refer_interval = refer_speed_ * mpc_param_.interval_;
    Eigen::Vector2d cur_pos = global_traj_.at(cur_idx);
    local_traj_.emplace_back(cur_pos[0], cur_pos[1]);
    // cause the policy of generating the m_ref_trajectory is unknown, need to
    // reformulate the trajectory
    for (size_t i = 0; i < mpc_param_.horizon_; ++i) {
      double dist = (global_traj_.at(cur_idx) - cur_pos).norm();
      while (dist < refer_interval && cur_idx + 1 < global_traj_.size()) {
        ++cur_idx;
        dist +=
            (global_traj_.at(cur_idx) - global_traj_.at(cur_idx - 1)).norm();
      }
      if (dist > refer_interval) {
        double res = dist - refer_interval,
               seg_dist =
                   (global_traj_.at(cur_idx) - global_traj_.at(cur_idx - 1))
                       .norm();
        cur_pos = global_traj_.at(cur_idx);
        cur_pos -= (res / seg_dist *
                    (global_traj_.at(cur_idx) - global_traj_.at(cur_idx - 1)));
        local_traj_.emplace_back(cur_pos[0], cur_pos[1]);
      } else {
        local_traj_.emplace_back(global_traj_.at(cur_idx)[0],
                                 global_traj_.at(cur_idx)[1]);
        break;
      }
    }
  }

  Eigen::Vector3d getNextState(const Eigen::Vector3d &current_state,
                               const Eigen::Vector2d &current_twist) {
    Eigen::Vector3d next_state;
    double angle = current_twist[1] > 0 ? current_state[2] - M_PI / 2
                                        : current_state[2] + M_PI / 2;
    double next_angle = current_twist[1] * mpc_param_.interval_ + angle;
    double radius = std::abs(current_twist[0] / current_twist[1]);
    double delta_x = std::cos(next_angle) * radius - std::cos(angle) * radius;
    double delta_y = std::sin(next_angle) * radius - std::sin(angle) * radius;
    double delta_theta = current_twist[1] * mpc_param_.interval_;

    next_state[0] = current_state[0] + delta_x;
    next_state[1] = current_state[1] + delta_y;
    next_state[2] = current_state[2] + delta_theta;
    return next_state;
  }
  void serialize() {
    ros::Time now = ros::Time::now();
    std::stringstream ss;
    ss << now.sec << "." << now.nsec;
    std::string time_stamp = ss.str();

    std::string file_name = file_path_ + "tracking_data_" + time_stamp;
    std::ofstream output_file(file_name, std::ios::out | std::ios::binary);
    if (!tracking_data_.SerializeToOstream(&output_file)) {
      ROS_ERROR("Failed to write tracking data to file.");
    } else {
      ROS_INFO("Tracking data has been written to file %s", file_name.c_str());
    }
    output_file.close();
  }
};
};  // namespace willand_ackermann
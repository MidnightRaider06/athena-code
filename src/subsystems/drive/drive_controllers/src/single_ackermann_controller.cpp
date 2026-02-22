// Copyright (c) 2025, UMDLoop
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "athena_drive_controllers/single_ackermann_controller.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <numeric>

#include "controller_interface/helpers.hpp"
#include "rclcpp/rclcpp.hpp"

namespace drive_controllers
{
SingleAckermannController::SingleAckermannController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn SingleAckermannController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<single_ackermann_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SingleAckermannController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  drive_joint_names_ = params_.drive_joints;
  steer_joint_names_ = params_.steer_joints;

  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&SingleAckermannController::reference_callback, this, std::placeholders::_1));

  input_ref_.writeFromNonRT(std::make_shared<ControllerReferenceMsg>());

  // Create odometry publishers
  odom_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    "~/odom", rclcpp::SystemDefaultsQoS());
  tf_odom_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
    "~/tf_odometry", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void SingleAckermannController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  input_ref_.writeFromNonRT(msg);
}

controller_interface::InterfaceConfiguration SingleAckermannController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : steer_joint_names_)
  {
    command_interfaces_config.names.push_back(joint + "/position");
  }
  for (const auto & joint : drive_joint_names_)
  {
    command_interfaces_config.names.push_back(joint + "/velocity");
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration SingleAckermannController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : steer_joint_names_)
  {
    state_interfaces_config.names.push_back(joint + "/position");
  }
  for (const auto & joint : drive_joint_names_)
  {
    state_interfaces_config.names.push_back(joint + "/velocity");
  }

  return state_interfaces_config;
}

bool SingleAckermannController::find_state_interface_index(
  const std::string & name, size_t & index) const
{
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    if (state_interfaces_[i].get_name() == name) {
      index = i;
      return true;
    }
  }
  return false;
}

controller_interface::CallbackReturn SingleAckermannController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialize all command interfaces to zero to prevent initial movement
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(0.0);
  }

  // Reset odometry state
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_heading_ = 0.0;
  prev_time_set_ = false;
  linear_vel_buffer_.clear();
  angular_vel_buffer_.clear();

  RCLCPP_INFO(get_node()->get_logger(), "SingleAckermannController activated with all commands set to zero");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SingleAckermannController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(0.0);
  }

  // Reset odometry state
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_heading_ = 0.0;
  prev_time_set_ = false;
  linear_vel_buffer_.clear();
  angular_vel_buffer_.clear();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SingleAckermannController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto current_ref = input_ref_.readFromRT();
  if (!current_ref || !(*current_ref))
  {
    // Set all command interfaces to zero when no input is available
    for (size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      command_interfaces_[i].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  // Clamp linear velocity first so steer angle is computed from actual execution speed
  double linear_vel_cmd = std::clamp(
    (*current_ref)->twist.linear.x, -params_.max_speed, params_.max_speed);

  // Compute steer angle from Twist using bicycle model: delta = atan(omega * L / v)
  double steer_cmd = 0.0;
  if (std::abs(linear_vel_cmd) > 1e-4) {
    steer_cmd = std::atan((*current_ref)->twist.angular.z * params_.wheelbase / linear_vel_cmd);
  }
  steer_cmd = std::clamp(steer_cmd, -params_.max_steer_angle, params_.max_steer_angle);

  double wheelbase = params_.wheelbase;
  double track_width = params_.track_width;
  double wheel_radius = params_.wheel_radius;

  double front_left_steer_angle = 0.0;
  double front_right_steer_angle = 0.0;

  double front_left_vel = linear_vel_cmd;
  double front_right_vel = linear_vel_cmd;
  double rear_left_vel = linear_vel_cmd;
  double rear_right_vel = linear_vel_cmd;

  // If we are turning...
  if (std::abs(steer_cmd) > 1e-4) {
    double turn_radius = wheelbase / std::tan(steer_cmd);
    double angular_vel = std::abs(linear_vel_cmd) / std::abs(turn_radius);

    // Preserve the sign of linear velocity for forward/backward motion
    if (linear_vel_cmd < 0) {
      angular_vel = -angular_vel;
    }

    // Calculate magnitudes of inner and outer wheel angles
    double inner_angle = std::atan(wheelbase / (std::abs(turn_radius) - track_width / 2.0));
    double outer_angle = std::atan(wheelbase / (std::abs(turn_radius) + track_width / 2.0));

    // Calculate magnitudes of wheel speeds
    double inner_rear_vel = angular_vel * (std::abs(turn_radius) - track_width / 2.0);
    double outer_rear_vel = angular_vel * (std::abs(turn_radius) + track_width / 2.0);
    double inner_front_vel = angular_vel * std::sqrt(
      std::pow(wheelbase, 2) + std::pow(std::abs(turn_radius) - track_width / 2.0, 2));
    double outer_front_vel = angular_vel * std::sqrt(
      std::pow(wheelbase, 2) + std::pow(std::abs(turn_radius) + track_width / 2.0, 2));

    // Assign angles and velocities based on the hardware's actual behavior
    if (steer_cmd > 0.0) {  // LEFT TURN: left wheel is INNER
      front_left_steer_angle = -inner_angle;
      front_right_steer_angle = -outer_angle;

      front_left_vel = inner_front_vel;
      front_right_vel = outer_front_vel;
      rear_left_vel = inner_rear_vel;
      rear_right_vel = outer_rear_vel;

    } else {  // RIGHT TURN: right wheel is INNER
      front_left_steer_angle = outer_angle;
      front_right_steer_angle = inner_angle;

      front_left_vel = outer_front_vel;
      front_right_vel = inner_front_vel;
      rear_left_vel = outer_rear_vel;
      rear_right_vel = inner_rear_vel;
    }
  }

  // Convert linear wheel velocities to angular velocities (rad/s)
  double fl_wheel_ang_vel = front_left_vel / wheel_radius;
  double fr_wheel_ang_vel = front_right_vel / wheel_radius;
  double rl_wheel_ang_vel = rear_left_vel / wheel_radius;
  double rr_wheel_ang_vel = rear_right_vel / wheel_radius;

  // Set steering positions
  command_interfaces_[0].set_value(front_left_steer_angle);
  command_interfaces_[1].set_value(front_right_steer_angle);

  // Set drive velocities
  command_interfaces_[2].set_value(fl_wheel_ang_vel);
  command_interfaces_[3].set_value(fr_wheel_ang_vel);
  command_interfaces_[4].set_value(rl_wheel_ang_vel);
  command_interfaces_[5].set_value(rr_wheel_ang_vel);

  // ── Odometry computation ──────────────────────────────────────────────
  // Determine the robot's actual linear and angular velocity for odom.
  // open_loop = true  → use the commanded velocities (useful for testing)
  // open_loop = false → use actual encoder feedback from state interfaces

  double odom_linear_vel = 0.0;
  double odom_angular_vel = 0.0;

  if (params_.open_loop) {
    // Open-loop: use commanded rear wheel average and steer angle
    odom_linear_vel = (rear_left_vel + rear_right_vel) / 2.0;
    double avg_steer = (front_left_steer_angle + front_right_steer_angle) / 2.0;
    if (std::abs(wheelbase) > 1e-6) {
      odom_angular_vel = odom_linear_vel * std::tan(avg_steer) / wheelbase;
    }
  } else {
    // Closed-loop: read actual encoder feedback from state interfaces
    // Look up rear drive wheel velocities (angular, rad/s) by name
    size_t rl_vel_idx = 0, rr_vel_idx = 0;
    size_t fl_steer_idx = 0, fr_steer_idx = 0;

    bool have_feedback = true;
    if (!find_state_interface_index(steer_joint_names_[0] + "/position", fl_steer_idx) ||
        !find_state_interface_index(steer_joint_names_[1] + "/position", fr_steer_idx)) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "Could not find steer state interfaces, falling back to open-loop odom");
      have_feedback = false;
    }
    // Rear drive joints are at indices [2] and [3] in drive_joint_names_ (bl, br)
    if (!find_state_interface_index(drive_joint_names_[2] + "/velocity", rl_vel_idx) ||
        !find_state_interface_index(drive_joint_names_[3] + "/velocity", rr_vel_idx)) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "Could not find rear drive state interfaces, falling back to open-loop odom");
      have_feedback = false;
    }

    if (have_feedback) {
      // Convert angular wheel velocity (rad/s) to linear (m/s)
      double rl_linear = state_interfaces_[rl_vel_idx].get_value() * wheel_radius;
      double rr_linear = state_interfaces_[rr_vel_idx].get_value() * wheel_radius;
      odom_linear_vel = (rl_linear + rr_linear) / 2.0;

      // Average actual steer angle from encoders
      double actual_fl_steer = state_interfaces_[fl_steer_idx].get_value();
      double actual_fr_steer = state_interfaces_[fr_steer_idx].get_value();
      double avg_steer = (actual_fl_steer + actual_fr_steer) / 2.0;

      if (std::abs(wheelbase) > 1e-6) {
        odom_angular_vel = odom_linear_vel * std::tan(avg_steer) / wheelbase;
      }
    } else {
      // Fallback to open-loop if state interfaces not available
      odom_linear_vel = (rear_left_vel + rear_right_vel) / 2.0;
      double avg_steer = (front_left_steer_angle + front_right_steer_angle) / 2.0;
      if (std::abs(wheelbase) > 1e-6) {
        odom_angular_vel = odom_linear_vel * std::tan(avg_steer) / wheelbase;
      }
    }
  }

  // Rolling average for velocity smoothing
  size_t window = static_cast<size_t>(std::max<int64_t>(1, params_.velocity_rolling_window_size));
  linear_vel_buffer_.push_back(odom_linear_vel);
  angular_vel_buffer_.push_back(odom_angular_vel);
  while (linear_vel_buffer_.size() > window) { linear_vel_buffer_.pop_front(); }
  while (angular_vel_buffer_.size() > window) { angular_vel_buffer_.pop_front(); }

  double smoothed_linear = std::accumulate(
    linear_vel_buffer_.begin(), linear_vel_buffer_.end(), 0.0) /
    static_cast<double>(linear_vel_buffer_.size());
  double smoothed_angular = std::accumulate(
    angular_vel_buffer_.begin(), angular_vel_buffer_.end(), 0.0) /
    static_cast<double>(angular_vel_buffer_.size());

  // Integrate pose using midpoint method
  if (prev_time_set_) {
    double dt = (time - prev_time_).seconds();
    if (dt > 0.0 && dt < 1.0) {  // Sanity check: skip if dt is negative or huge
      double heading_mid = odom_heading_ + smoothed_angular * dt / 2.0;
      odom_x_ += smoothed_linear * std::cos(heading_mid) * dt;
      odom_y_ += smoothed_linear * std::sin(heading_mid) * dt;
      odom_heading_ += smoothed_angular * dt;
    }
  }
  prev_time_ = time;
  prev_time_set_ = true;

  // Publish nav_msgs/Odometry
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = time;
  odom_msg.header.frame_id = params_.odom_frame_id;
  odom_msg.child_frame_id = params_.base_frame_id;

  // Pose (position + orientation as quaternion from yaw)
  odom_msg.pose.pose.position.x = odom_x_;
  odom_msg.pose.pose.position.y = odom_y_;
  odom_msg.pose.pose.position.z = 0.0;
  // Convert heading to quaternion (rotation around Z axis only)
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = std::sin(odom_heading_ / 2.0);
  odom_msg.pose.pose.orientation.w = std::cos(odom_heading_ / 2.0);

  // Twist (in child_frame_id = base_link)
  odom_msg.twist.twist.linear.x = smoothed_linear;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = smoothed_angular;

  // Covariance (diagonal, reasonable defaults for wheeled robot on flat terrain)
  // Row-major 6x6: [x, y, z, roll, pitch, yaw]
  odom_msg.pose.covariance[0] = 0.01;   // x
  odom_msg.pose.covariance[7] = 0.01;   // y
  odom_msg.pose.covariance[14] = 1e6;   // z (not measured, set high)
  odom_msg.pose.covariance[21] = 1e6;   // roll
  odom_msg.pose.covariance[28] = 1e6;   // pitch
  odom_msg.pose.covariance[35] = 0.03;  // yaw

  odom_msg.twist.covariance[0] = 0.01;   // vx
  odom_msg.twist.covariance[7] = 0.01;   // vy
  odom_msg.twist.covariance[14] = 1e6;   // vz
  odom_msg.twist.covariance[21] = 1e6;   // roll rate
  odom_msg.twist.covariance[28] = 1e6;   // pitch rate
  odom_msg.twist.covariance[35] = 0.03;  // yaw rate

  odom_publisher_->publish(odom_msg);

  // Optionally publish TF (odom -> base_link)
  if (params_.enable_odom_tf) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = time;
    tf.header.frame_id = params_.odom_frame_id;
    tf.child_frame_id = params_.base_frame_id;
    tf.transform.translation.x = odom_x_;
    tf.transform.translation.y = odom_y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom_msg.pose.pose.orientation;

    tf2_msgs::msg::TFMessage tf_msg;
    tf_msg.transforms.push_back(tf);
    tf_odom_publisher_->publish(tf_msg);
  }

  return controller_interface::return_type::OK;
}
}  // namespace drive_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  drive_controllers::SingleAckermannController, controller_interface::ControllerInterface)

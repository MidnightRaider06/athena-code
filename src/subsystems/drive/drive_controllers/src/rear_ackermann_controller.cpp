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

#include "athena_drive_controllers/rear_ackermann_controller.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "controller_interface/helpers.hpp"
#include "rclcpp/rclcpp.hpp"

namespace drive_controllers
{
RearAckermannController::RearAckermannController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn RearAckermannController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<rear_ackermann_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RearAckermannController::on_configure(
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
    std::bind(&RearAckermannController::reference_callback, this, std::placeholders::_1));

  input_ref_.writeFromNonRT(std::make_shared<ControllerReferenceMsg>());

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void RearAckermannController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  input_ref_.writeFromNonRT(msg);
}

controller_interface::InterfaceConfiguration RearAckermannController::command_interface_configuration() const
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

controller_interface::InterfaceConfiguration RearAckermannController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return state_interfaces_config;
}

controller_interface::CallbackReturn RearAckermannController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(0.0);
  }

  RCLCPP_INFO(get_node()->get_logger(), "RearAckermannController activated with all commands set to zero");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RearAckermannController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RearAckermannController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto current_ref = input_ref_.readFromRT();
  if (!current_ref || !(*current_ref))
  {
    for (size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      command_interfaces_[i].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  double linear_vel_cmd = std::clamp(
    (*current_ref)->twist.linear.x, -params_.max_speed, params_.max_speed);

  // Bicycle model: same formula as front steering — sign of steer_cmd determines left vs right
  double steer_cmd = 0.0;
  if (std::abs(linear_vel_cmd) > 1e-4) {
    steer_cmd = std::atan((*current_ref)->twist.angular.z * params_.wheelbase / linear_vel_cmd);
  }
  //steer_cmd = std::clamp(steer_cmd, -params_.max_steer_angle, params_.max_steer_angle);

  double wheelbase = params_.wheelbase;
  double track_width = params_.track_width;
  double wheel_radius = params_.wheel_radius;

  double rear_left_steer_angle = 0.0;
  double rear_right_steer_angle = 0.0;

  double front_left_vel = linear_vel_cmd;
  double front_right_vel = linear_vel_cmd;
  double rear_left_vel = linear_vel_cmd;
  double rear_right_vel = linear_vel_cmd;

  if (std::abs(steer_cmd) > 1e-4) {
    double turn_radius = wheelbase / std::tan(steer_cmd);
    double angular_vel = std::abs(linear_vel_cmd) / std::abs(turn_radius);

    if (linear_vel_cmd < 0) {
      angular_vel = -angular_vel;
    }

    double inner_angle = std::atan(wheelbase / (std::abs(turn_radius) - track_width / 2.0));
    double outer_angle = std::atan(wheelbase / (std::abs(turn_radius) + track_width / 2.0));

    // Rear axle is the steered axle: it traces the longer (front) arc
    // Front axle is the fixed axle: it traces the shorter (rear) arc
    // Steer angles are negated vs front steering: rear wheels point right to turn left
    double inner_rear_vel = angular_vel * (std::abs(turn_radius) - track_width / 2.0);
    double outer_rear_vel = angular_vel * (std::abs(turn_radius) + track_width / 2.0);
    double inner_front_vel = angular_vel * std::sqrt(
      std::pow(wheelbase, 2) + std::pow(std::abs(turn_radius) - track_width / 2.0, 2));
    double outer_front_vel = angular_vel * std::sqrt(
      std::pow(wheelbase, 2) + std::pow(std::abs(turn_radius) + track_width / 2.0, 2));

    if (steer_cmd > 0.0) {  // LEFT TURN: left wheel is INNER
      rear_left_steer_angle = inner_angle;
      rear_right_steer_angle = outer_angle;

      front_left_vel = inner_rear_vel;
      front_right_vel = outer_rear_vel;
      rear_left_vel = inner_front_vel;
      rear_right_vel = outer_front_vel;

    } else {  // RIGHT TURN: right wheel is INNER
      rear_left_steer_angle = -outer_angle;
      rear_right_steer_angle = -inner_angle;

      front_left_vel = outer_rear_vel;
      front_right_vel = inner_rear_vel;
      rear_left_vel = outer_front_vel;
      rear_right_vel = inner_front_vel;
    }
  }

  double fl_wheel_ang_vel = front_left_vel / wheel_radius;
  double fr_wheel_ang_vel = front_right_vel / wheel_radius;
  double rl_wheel_ang_vel = rear_left_vel / wheel_radius;
  double rr_wheel_ang_vel = rear_right_vel / wheel_radius;

  // Set steering positions (rear wheels)

  double rear_left_angle_clamped = std::clamp(rear_left_steer_angle, -params_.max_steer_angle, params_.max_steer_angle);
  double rear_right_angle_clamped = -1* std::clamp(rear_right_steer_angle, -params_.max_steer_angle, params_.max_steer_angle);



  command_interfaces_[0].set_value(rear_left_angle_clamped);
  command_interfaces_[1].set_value(rear_right_angle_clamped);

  // Set drive velocities
  command_interfaces_[2].set_value(fl_wheel_ang_vel);
  command_interfaces_[3].set_value(fr_wheel_ang_vel);
  command_interfaces_[4].set_value(rl_wheel_ang_vel);
  command_interfaces_[5].set_value(rr_wheel_ang_vel);

  const double rad_s_to_rpm = 60.0 / (2.0 * M_PI);
  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
    "Wheel speeds [RPM] - FL: %.2f, FR: %.2f, RL: %.2f, RR: %.2f | Steer [rad] - RL: %.3f, RR: %.3f",
    fl_wheel_ang_vel * rad_s_to_rpm,
    fr_wheel_ang_vel * rad_s_to_rpm,
    rl_wheel_ang_vel * rad_s_to_rpm,
    rr_wheel_ang_vel * rad_s_to_rpm,
    rear_left_angle_clamped,
    rear_right_angle_clamped);

  return controller_interface::return_type::OK;
}
}  // namespace drive_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  drive_controllers::RearAckermannController, controller_interface::ControllerInterface)
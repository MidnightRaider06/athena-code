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

#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

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

controller_interface::CallbackReturn SingleAckermannController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initialize all command interfaces to zero to prevent initial movement
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(0.0);
  }
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
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SingleAckermannController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto current_ref = input_ref_.readFromRT();
  if (!current_ref || !(*current_ref) || (*current_ref)->axes.empty())
  {
    // Set all command interfaces to zero when no input is available
    for (size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      command_interfaces_[i].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  // Get joystick values and apply scaling and inversion from parameters
  double linear_vel_cmd = (*current_ref)->axes[params_.forward_axis] * params_.max_speed;
  double steer_cmd = (*current_ref)->axes[params_.steer_axis] * params_.max_steer_angle;
  if (params_.steer_inversion) {
    steer_cmd *= -1.0;
  }

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
    double turn_radius = wheelbase / tan(steer_cmd);
    double angular_vel = std::abs(linear_vel_cmd) / std::abs(turn_radius);
    
    // Preserve the sign of linear velocity for forward/backward motion
    if (linear_vel_cmd < 0) {
      angular_vel = -angular_vel;
    }

    // Calculate magnitudes of inner and outer wheel angles
    double inner_angle = atan(wheelbase / (std::abs(turn_radius) - track_width / 2.0));
    double outer_angle = atan(wheelbase / (std::abs(turn_radius) + track_width / 2.0));
    
    // Calculate magnitudes of wheel speeds
    double inner_rear_vel = angular_vel * (std::abs(turn_radius) - track_width / 2.0);
    double outer_rear_vel = angular_vel * (std::abs(turn_radius) + track_width / 2.0);
    double inner_front_vel = angular_vel * sqrt(pow(wheelbase, 2) + pow(std::abs(turn_radius) - track_width / 2.0, 2));
    double outer_front_vel = angular_vel * sqrt(pow(wheelbase, 2) + pow(std::abs(turn_radius) + track_width / 2.0, 2));

    // Assign angles and velocities based on the hardware's actual behavior
    if (steer_cmd > 0.0) { // LEFT TURN: left wheel is INNER
      front_left_steer_angle = -inner_angle;
      front_right_steer_angle = -outer_angle;
      
      front_left_vel = inner_front_vel;
      front_right_vel = outer_front_vel;
      rear_left_vel = inner_rear_vel;
      rear_right_vel = outer_rear_vel;

    } else { // RIGHT TURN: right wheel is INNER
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

  return controller_interface::return_type::OK;
}
}  // namespace drive_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  drive_controllers::SingleAckermannController, controller_interface::ControllerInterface)

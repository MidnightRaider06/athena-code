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

#include "general_controllers/motor_status_broadcaster.hpp"

#include <sstream>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace general_controllers
{

MotorStatusBroadcaster::MotorStatusBroadcaster() {}

controller_interface::CallbackReturn MotorStatusBroadcaster::on_init()
{
  try {
    param_listener_ = std::make_shared<motor_status_broadcaster::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MotorStatusBroadcaster::command_interface_configuration() const
{
  // Broadcaster only reads, no command interfaces
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
MotorStatusBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : params_.joints) {
    for (const auto & iface : params_.interfaces) {
      config.names.push_back(joint + "/" + iface);
    }
  }

  return config;
}

controller_interface::CallbackReturn MotorStatusBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified for motor_status_broadcaster.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.interfaces.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No interfaces specified for motor_status_broadcaster.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Set up publish rate
  if (params_.publish_rate > 0.0) {
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / params_.publish_rate);
  } else {
    publish_period_ = rclcpp::Duration(0, 0);
  }

  diagnostics_publisher_ = get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "~/diagnostics", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured motor_status_broadcaster for %zu joints, %zu interfaces, publish rate: %.1f Hz",
    params_.joints.size(), params_.interfaces.size(), params_.publish_rate);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotorStatusBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Build lookup map from "joint/interface" -> state_interfaces_ index
  state_interface_map_.clear();
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    state_interface_map_[state_interfaces_[i].get_prefix_name() + "/" +
                         state_interfaces_[i].get_interface_name()] = i;
  }

  last_publish_time_ = get_node()->now();

  RCLCPP_INFO(
    get_node()->get_logger(),
    "MotorStatusBroadcaster activated with %zu state interfaces",
    state_interfaces_.size());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotorStatusBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  state_interface_map_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MotorStatusBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Rate limit publishing
  if (publish_period_.seconds() > 0.0 && (time - last_publish_time_) < publish_period_) {
    return controller_interface::return_type::OK;
  }
  last_publish_time_ = time;

  diagnostic_msgs::msg::DiagnosticArray diag_msg;
  diag_msg.header.stamp = time;

  for (const auto & joint : params_.joints) {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "Motor: " + joint;
    status.hardware_id = joint;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "OK";

    for (const auto & iface : params_.interfaces) {
      std::string key = joint + "/" + iface;
      auto it = state_interface_map_.find(key);

      diagnostic_msgs::msg::KeyValue kv;
      kv.key = iface;

      if (it != state_interface_map_.end()) {
        double value = state_interfaces_[it->second].get_value();

        // Format value with 2 decimal places
        std::ostringstream oss;
        oss.precision(2);
        oss << std::fixed << value;
        kv.value = oss.str();

        // Flag high temperature as warning (>70C) or error (>90C)
        if (iface == "motor_temperature") {
          if (value > 90.0) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "OVERHEATING";
          } else if (value > 70.0 &&
                     status.level < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = "High temperature";
          }
        }
      } else {
        kv.value = "N/A";
      }

      status.values.push_back(kv);
    }

    diag_msg.status.push_back(status);
  }

  diagnostics_publisher_->publish(diag_msg);

  return controller_interface::return_type::OK;
}

}  // namespace general_controllers

PLUGINLIB_EXPORT_CLASS(
  general_controllers::MotorStatusBroadcaster,
  controller_interface::ControllerInterface)

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

#ifndef GENERAL_CONTROLLERS__MOTOR_STATUS_BROADCASTER_HPP_
#define GENERAL_CONTROLLERS__MOTOR_STATUS_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "general_controllers/visibility_control.h"
#include <general_controllers/motor_status_broadcaster_parameters.hpp>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace general_controllers
{

class MotorStatusBroadcaster : public controller_interface::ControllerInterface
{
public:
  GENERAL_CONTROLLERS__VISIBILITY_PUBLIC
  MotorStatusBroadcaster();

  GENERAL_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  GENERAL_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  GENERAL_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  GENERAL_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  GENERAL_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  GENERAL_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  GENERAL_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<motor_status_broadcaster::ParamListener> param_listener_;
  motor_status_broadcaster::Params params_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;

  // Map from "joint/interface" to index in state_interfaces_
  std::unordered_map<std::string, size_t> state_interface_map_;

  // Publish rate limiting
  rclcpp::Duration publish_period_{0, 0};
  rclcpp::Time last_publish_time_{0, 0, RCL_CLOCK_UNINITIALIZED};
};

}  // namespace general_controllers

#endif  // GENERAL_CONTROLLERS__MOTOR_STATUS_BROADCASTER_HPP_

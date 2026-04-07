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

#ifndef ATHENA_DRIVE_CONTROLLERS__REAR_ACKERMANN_CONTROLLER_HPP_
#define ATHENA_DRIVE_CONTROLLERS__REAR_ACKERMANN_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include <drive_controllers/rear_ackermann_controller_parameters.hpp>
#include "athena_drive_controllers/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace drive_controllers
{

class RearAckermannController : public controller_interface::ControllerInterface
{
public:
  ATHENA_DRIVE_CONTROLLERS__VISIBILITY_PUBLIC
  RearAckermannController();

  ATHENA_DRIVE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  ATHENA_DRIVE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  ATHENA_DRIVE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ATHENA_DRIVE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ATHENA_DRIVE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ATHENA_DRIVE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ATHENA_DRIVE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using ControllerReferenceMsg = geometry_msgs::msg::TwistStamped;

protected:
  std::shared_ptr<rear_ackermann_controller::ParamListener> param_listener_;
  rear_ackermann_controller::Params params_;

  std::vector<std::string> drive_joint_names_;
  std::vector<std::string> steer_joint_names_;

  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

private:
  ATHENA_DRIVE_CONTROLLERS__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
};

}  // namespace drive_controllers

#endif  // ATHENA_DRIVE_CONTROLLERS__REAR_ACKERMANN_CONTROLLER_HPP_
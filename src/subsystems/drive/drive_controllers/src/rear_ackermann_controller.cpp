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
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "rclcpp/rclcpp.hpp"

namespace drive_controllers
{

RearAckermannController::RearAckermannController()
: controller_interface::ControllerInterface() {}

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
  steer_joint_names_ = params_.steer_joints;  // [fl, fr, bl, br]
  drive_joint_names_ = params_.drive_joints;  // [fl, fr, bl, br]

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

void RearAckermannController::reference_callback(
  const std::shared_ptr<ControllerReferenceMsg> msg)
{
  input_ref_.writeFromNonRT(msg);
}

controller_interface::InterfaceConfiguration
RearAckermannController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Steer interfaces: [fl, fr, bl, br] / position
  for (const auto & joint : steer_joint_names_) {
    config.names.push_back(joint + "/position");
  }
  // Drive interfaces: [fl, fr, bl, br] / velocity
  for (const auto & joint : drive_joint_names_) {
    config.names.push_back(joint + "/velocity");
  }

  return config;
}

controller_interface::InterfaceConfiguration
RearAckermannController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::CallbackReturn RearAckermannController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_[i].set_value(0.0);
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "RearAckermannController activated with all commands set to zero");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RearAckermannController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_[i].set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RearAckermannController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto current_ref = input_ref_.readFromRT();
  if (!current_ref || !(*current_ref)) {
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
      command_interfaces_[i].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  const double v = std::clamp(
    (*current_ref)->twist.linear.x, -params_.max_speed, params_.max_speed);
  const double omega = (*current_ref)->twist.angular.z;

  const double wheelbase    = params_.wheelbase;
  const double track_width  = params_.track_width;
  const double wheel_radius = params_.wheel_radius;
  const double half_base    = wheelbase / 2.0;
  const double half_track   = track_width / 2.0;

  // command_interfaces_ layout:
  //   [0] steer_fl / position  ← held at 0 (front tank: wheels point straight)
  //   [1] steer_fr / position  ← held at 0
  //   [2] steer_bl / position  ← rear swerve angle
  //   [3] steer_br / position  ← rear swerve angle
  //   [4] propulsion_fl / velocity  ← front Ackermann arc speed
  //   [5] propulsion_fr / velocity  ← front Ackermann arc speed
  //   [6] propulsion_bl / velocity  ← rear Ackermann arc speed
  //   [7] propulsion_br / velocity  ← rear Ackermann arc speed

  if (std::abs(v) < 1e-4 && std::abs(omega) < 1e-4) {
    // Stationary: zero everything
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
      command_interfaces_[i].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  if (std::abs(omega) < 1e-4) {
    // Straight line: all steer = 0, all drive = v / r
    const double drive_ang = v / wheel_radius;
    command_interfaces_[0].set_value(0.0);
    command_interfaces_[1].set_value(0.0);
    command_interfaces_[2].set_value(0.0);
    command_interfaces_[3].set_value(0.0);
    command_interfaces_[4].set_value(drive_ang);
    command_interfaces_[5].set_value(drive_ang);
    command_interfaces_[6].set_value(drive_ang);
    command_interfaces_[7].set_value(drive_ang);

    const double rad_s_to_rpm = 60.0 / (2.0 * M_PI);
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
      "Wheel speeds [RPM] - FL: %.2f, FR: %.2f, RL: %.2f, RR: %.2f",
      drive_ang * rad_s_to_rpm,
      drive_ang * rad_s_to_rpm,
      drive_ang * rad_s_to_rpm,
      drive_ang * rad_s_to_rpm);
    return controller_interface::return_type::OK;
  }

  // ── Turn geometry (mid-vehicle ICR, consistent with double_ackermann_controller) ──
  //
  // ICR is at lateral distance R = v / omega from the vehicle centreline,
  // centred between the axles.  Left-hand turns have R > 0.
  const double turn_radius = v / omega;                       // R  (signed)
  const double r_left      = turn_radius - half_track;        // R − T/2
  const double r_right     = turn_radius + half_track;        // R + T/2

  // ── Rear wheels: swerve (steer + Ackermann arc speed) ───────────────────
  //
  // Steer angle: rear wheels counter-steer relative to what front wheels would do.
  // angle = -atan(half_base / r_side): legs of the right triangle are the longitudinal
  // offset (half_base) and the lateral ICR distance (r_side), so atan is correct here.
  const double rear_left_steer  = -std::atan(half_base / r_left);
  const double rear_right_steer = -std::atan(half_base / r_right);

  // Arc speed: r * omega (signed — correct for forward and reverse)
  const double rear_left_vel  = (r_right * omega) / wheel_radius; //flipped on purpose (just for sim?)
  const double rear_right_vel = (r_left  * omega) / wheel_radius; //flipped on purpose (just for sim?)

  // ── Front wheels: pure Ackermann arc speed ───────────────────────────────
  //
  // The front steer joints are held at 0, so the kinematically correct roll
  // speed is r_side * omega / r_w.  The inner wheel transitions from forward
  // to backward only when |R| < half_track (very tight turns).
  const double front_left_vel  = (r_right * omega) / wheel_radius; //flipped on purpose
  const double front_right_vel = (r_left  * omega) / wheel_radius; //flipped on purpose

  // Front steer joints always 0 (wheels point straight)
  command_interfaces_[0].set_value(0.0);
  command_interfaces_[1].set_value(0.0);
  command_interfaces_[2].set_value(rear_left_steer);
  command_interfaces_[3].set_value(rear_right_steer);
  command_interfaces_[4].set_value(front_left_vel);
  command_interfaces_[5].set_value(front_right_vel);
  command_interfaces_[6].set_value(rear_left_vel);
  command_interfaces_[7].set_value(rear_right_vel);

  const double rad_s_to_rpm = 60.0 / (2.0 * M_PI);
  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
    "Wheel speeds [RPM] - FL: %.2f, FR: %.2f, RL: %.2f, RR: %.2f",
    front_left_vel * rad_s_to_rpm,
    front_right_vel * rad_s_to_rpm,
    rear_left_vel * rad_s_to_rpm,
    rear_right_vel * rad_s_to_rpm);

  return controller_interface::return_type::OK;
}

}  // namespace drive_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  drive_controllers::RearAckermannController, controller_interface::ControllerInterface)

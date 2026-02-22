// Copyright (c) 2025, UMDLoop
// Copyright (c) 2025, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#include "athena_drive_controllers/double_ackermann_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

using ControllerReferenceMsg = drive_controllers::DoubleAckermannController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  std::shared_ptr<ControllerReferenceMsg> & msg)
{
  msg->linear.x = std::numeric_limits<double>::quiet_NaN();
  msg->linear.y = std::numeric_limits<double>::quiet_NaN();
  msg->linear.z = std::numeric_limits<double>::quiet_NaN();
  msg->angular.x = std::numeric_limits<double>::quiet_NaN();
  msg->angular.y = std::numeric_limits<double>::quiet_NaN();
  msg->angular.z = std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

namespace drive_controllers
{
DoubleAckermannController::DoubleAckermannController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn DoubleAckermannController::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);

  try
  {
    param_listener_ = std::make_shared<double_ackermann_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DoubleAckermannController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  drive_joints.push_back(params_.front_left_drive_joint);
  drive_joints.push_back(params_.front_right_drive_joint);
  drive_joints.push_back(params_.rear_left_drive_joint);
  drive_joints.push_back(params_.rear_right_drive_joint);
  swerve_joints.push_back(params_.front_left_swerve_joint);
  swerve_joints.push_back(params_.front_right_swerve_joint);
  swerve_joints.push_back(params_.rear_left_swerve_joint);
  swerve_joints.push_back(params_.rear_right_swerve_joint);

  joints.insert(joints.end(), drive_joints.begin(), drive_joints.end());
  joints.insert(joints.end(), swerve_joints.begin(), swerve_joints.end());

  if (!params_.state_joints.empty())
  {
    state_joints_ = params_.state_joints;
  }
  else
  {
    state_joints_ = joints;
  }

  if (joints.size() != state_joints_.size())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'joints' (%zu) and 'state_joints' (%zu) parameters has to be the same!",
      joints.size(), state_joints_.size());
    return CallbackReturn::FAILURE;
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&DoubleAckermannController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg);
  input_ref_.writeFromNonRT(msg);

  auto set_slow_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response)
  {
    if (request->data)
    {
      control_mode_.writeFromNonRT(control_mode_type::SLOW);
    }
    else
    {
      control_mode_.writeFromNonRT(control_mode_type::FAST);
    }
    response->success = true;
  };

  set_slow_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_slow_control_mode", set_slow_mode_service_callback,
    rmw_qos_profile_services_hist_keep_all);

  try
  {
    // State publisher
    s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // TODO(anyone): Reserve memory in state publisher depending on the message type
  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = joints[0];
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void DoubleAckermannController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> /*msg*/)
{
  // if (msg->joint_names.size() == joints.size())
  // {
  //   input_ref_.writeFromNonRT(msg);
  // }
  // else
  // {
  //   RCLCPP_ERROR(
  //     get_node()->get_logger(),
  //     "Received %zu , but expected %zu joints in command. Ignoring message.",
  //     msg->joint_names.size(), joints.size());
  // }
}

controller_interface::InterfaceConfiguration DoubleAckermannController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(joints.size());
  for (const auto & joint : swerve_joints)
  {
    command_interfaces_config.names.push_back(joint + "/" + "position");
  }
  for (const auto & joint : drive_joints)
  {
    command_interfaces_config.names.push_back(joint + "/" + "velocity");
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration DoubleAckermannController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joints_.size());
  for (const auto & joint : swerve_joints)
  {
    state_interfaces_config.names.push_back(joint + "/" + "position");
  }
  for (const auto & joint : drive_joints)
  {
    state_interfaces_config.names.push_back(joint + "/" + "velocity");
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn DoubleAckermannController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
  // `on_activate` method in `JointTrajectoryController` for exemplary use of
  // `controller_interface::get_ordered_interfaces` helper function

  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT)());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DoubleAckermannController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DoubleAckermannController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto current_ref = input_ref_.readFromRT();

  double linear_x = (*current_ref)->linear.x;
  double angular_z = (*current_ref)->angular.z;
  double track_width = params_.track_width;
  double wheelbase = params_.wheelbase;
  double wheel_radius = params_.wheel_radius;

  // WHen the turning radius is negative, the icr is on the right side, which ultimately leads to all the
  // following calculaitons working out even if radius can never actually be negative
  double turning_radius = linear_x/angular_z;
  double r_left = turning_radius - (track_width/2);
  double r_right = turning_radius + (track_width/2);

  double left_wheel_velocity = (r_left*angular_z)/wheel_radius;
  double right_wheel_velocity = (r_right*angular_z)/wheel_radius;;

  double front_left_position = std::asin((wheelbase/2)/r_left);
  double front_right_position = std::asin(((wheelbase/2)/r_right));
  double rear_left_position = std::asin(-(wheelbase/2)/r_left);
  double rear_right_position = std::asin(-(wheelbase/2)/r_right);


  command_interfaces_[0].set_value(front_left_position);
  command_interfaces_[1].set_value(front_right_position);
  command_interfaces_[2].set_value(rear_left_position);
  command_interfaces_[3].set_value(rear_right_position);
  command_interfaces_[4].set_value(left_wheel_velocity);
  command_interfaces_[5].set_value(right_wheel_velocity);
  command_interfaces_[6].set_value(left_wheel_velocity);
  command_interfaces_[7].set_value(right_wheel_velocity);


  (*current_ref)->linear.x = std::numeric_limits<double>::quiet_NaN();
  (*current_ref)->linear.y = std::numeric_limits<double>::quiet_NaN();
  (*current_ref)->linear.z = std::numeric_limits<double>::quiet_NaN();
  (*current_ref)->angular.x = std::numeric_limits<double>::quiet_NaN();
  (*current_ref)->angular.y = std::numeric_limits<double>::quiet_NaN();
  (*current_ref)->angular.z = std::numeric_limits<double>::quiet_NaN();

  if (state_publisher_ && state_publisher_->trylock())
  {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.set_point = command_interfaces_[CMD_MY_ITFS].get_value();
    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace drive_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  drive_controllers::DoubleAckermannController, controller_interface::ControllerInterface)

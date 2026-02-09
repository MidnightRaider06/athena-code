// Copyright (c) 2025, UMDLoop
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#ifndef SCIENCE_CONTROLLERS__SCIENCE_MANUAL_HPP_
#define SCIENCE_CONTROLLERS__SCIENCE_MANUAL_HPP_

#include <memory>
#include <string>
#include <vector>

#include "science_controllers/science_manual_parameters.hpp"
#include "controller_interface/controller_interface.hpp"
#include "science_controllers/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "control_msgs/msg/joint_controller_state.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace science_controllers
{

// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = 0;

// CONTROL MODE FOR DIFFERENT STAGES IN SCIENCE
enum class control_mode_type : std::uint8_t
{
  STAGE1 = 0,
  STAGE2 = 1,
  STAGE3 = 2,
  STAGE4 = 3
};

enum class control_speed_type : std::uint8_t
{
  VERY_SLOW = 0,
  SLOW = 1,
  MEDIUM = 2,
  FAST = 3
};

class ScienceManual : public controller_interface::ControllerInterface
{
public:
  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  ScienceManual();

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // TODO(anyone): replace the state and command message types
  using ControllerReferenceMsg = sensor_msgs::msg::Joy;
  using ControllerModeSrvType = std_srvs::srv::SetBool;
  using ControllerStateMsg = control_msgs::msg::JointControllerState;


protected:

  std::shared_ptr<science_manual::ParamListener> param_listener_;
  science_manual::Params params_;

  std::vector<std::string> state_joints_;
  std::vector<std::string> stepper_joints_;
  std::vector<std::string> talon_joints_;
  std::vector<std::string> servo_joints_;
  std::vector<std::string> rack_pinion_joints_;
  //std::string auger_spinner_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  rclcpp::Service<ControllerModeSrvType>::SharedPtr set_slow_control_mode_service_;
  realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

private:
  // callback for topic interface
  SCIENCE_CONTROLLERS__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);

  std::vector<int32_t> prev_buttons_;

  control_mode_type current_mode_{control_mode_type::STAGE1};
  void load_velocity_limits();  // called in on_configure()

  void send_commands(
    double lift_cmd,
    double stepper_cmd,
    double scoop_cmd,
    double auger_cmd,
    double rack_left_cmd,
    double rack_right_cmd
    //double auger_spinner_cmd
    );
  };

  static constexpr double max_lift_velocity = 1.0;
  static constexpr double max_stepper_velocity = 1.0;
  static constexpr double scoop_talon_velocity = 1.0;
  static constexpr double auger_velocity = 1.0;
  
  // Closed = 0, Open = 1
  double scoop_position = 0;
  double auger_position = 0;
  double cap_position = 0;
  double rack_left_position = 0.0;
  double rack_right_position = 0.0;

  /*enum CommandInterfaces
  {
    IDX_LIFT_TALON_VELOCITY = 0,
    IDX_STEPPERS_VELOCITY_START,  
    IDX_SCOOP_TALON_VELOCITY,
    IDX_AUGER_VELOCITY,
    IDX_SCOOP_SERVO_POSITION,
    IDX_STEPPERS_PUMPING_MODE,
    IDX_AUGER_SERVO_POSITION,
    IDX_CAP_SERVO_POSITION,
    CMD_ITFS_COUNT  
  }; */

  enum CommandInterfaces
  {
    // ----- Steppers (position control) -----
    IDX_STEPPER_A_POSITION = 0,
    IDX_STEPPER_B_POSITION = 1,

    // ----- Talons (velocity control) -----
    IDX_LIFT_TALON_VELOCITY = 2,
    IDX_SCOOP_TALON_VELOCITY = 3,

    // ----- Scoop servos (position) -----
    IDX_SCOOP_A_POSITION = 4,
    IDX_SCOOP_B_POSITION = 5,

    // ----- Auger servo -----
    IDX_AUGER_POSITION = 6,

    // ----- Cap servo -----
    IDX_CAP_POSITION = 7,

    // ----- Rack and Pinion servos -----
    IDX_RACK_LEFT_POSITION  = 8,
    IDX_RACK_RIGHT_POSITION = 9,
    
    // Total number of interfaces
    CMD_ITFS_COUNT
  };

  
};

// namespace science_controllers

#endif  // SCIENCE_CONTROLLERS__SCIENCE_MANUAL_HPP_

// Copyright (c) 2024 UMD Loop
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

#ifndef STEPPER_HARDWARE_INTERFACE_HPP_
#define STEPPER_HARDWARE_INTERFACE_HPP_

#include <netinet/in.h>

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <cstdint>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "umdloop_can_library/CanFrame.hpp"
#include "umdloop_can_library/SocketCanBus.hpp"

namespace CANLib
{
struct CanFrame;
}

namespace stepper_ros2_control
{
class STEPPERHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(STEPPERHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  enum class integration_level_t : std::uint8_t
  {
    UNDEFINED = 0,
    POSITION = 1,
    VELOCITY = 2,
  };

  struct StepperJoint
  {
    std::string name;
    uint8_t node_id;
    int gear_ratio;
    int orientation;
    integration_level_t control_level;

    double joint_state_position;
    double joint_state_velocity;
    double motor_position;
    double motor_velocity;
    double motor_status;
    double motor_temperature;
    double motor_torque_current;

    double joint_command_position;
    double joint_command_velocity;
    double motor_status_req;
    double motor_maintenance_req;
    double maintenance_frame_high;
    double maintenance_frame_low;
    double maintenance_frame;
    double maintenance_data_count;
    std::vector<uint8_t> decoded_maintenance_frame;

    double prev_status_req;
    double prev_maintenance_req;
    double elapsed_status_request_time;
    double elapsed_maintenance_request_time;
    double prev_joint_command_position;
    double prev_joint_command_velocity;

    std::vector<std::string> state_interface_names;
    std::vector<std::string> command_interface_names;
    std::unordered_map<std::string, std::string> parameters;
  };

  struct DecodedCommand
  {
    uint8_t command_id;
    std::vector<uint8_t> u8_data;
    std::vector<int16_t> i16_data;
    std::vector<int32_t> i32_data;
  };

  void on_can_message(const CANLib::CanFrame & frame);
  void logger_function();

  double calculate_joint_position_from_motor_position(double motor_position, int gear_ratio);
  double calculate_joint_velocity_from_motor_velocity(double motor_velocity, int gear_ratio);
  int16_t calculate_motor_position_from_desired_joint_position(double joint_position, int gear_ratio);
  int16_t calculate_motor_velocity_from_desired_joint_velocity(double joint_velocity, int gear_ratio);

  void format_control_command(CANLib::CanFrame & frame, StepperJoint & joint);
  bool format_status_command(CANLib::CanFrame & frame, uint8_t command_id, uint8_t node_id);
  bool format_maintenance_command(
    CANLib::CanFrame & frame, uint8_t node_id, const DecodedCommand & decoded_cmd);

private:
  int update_rate;
  double elapsed_update_time;
  double elapsed_time;
  double elapsed_logger_time;
  int logger_rate;
  int logger_state;
  int write_count;
  int num_joints;

  int can_command_id;
  uint32_t can_response_id;
  CANLib::SocketCanBus canBus;
  CANLib::CanFrame can_tx_frame_;
  CANLib::CanFrame can_rx_frame_;
  std::string can_interface;

  std::vector<StepperJoint> STEPPERJoints_;

  enum class ControlCommands : uint8_t
  {
    RELATIVE_POS_CONTROL_CMD = 0x20,
    VELOCITY_CONTROL_CMD = 0x30,
  };

  enum class MaintenanceCommands : uint8_t
  {
    PCB_HEARTBEAT_CMD = 0x10,
    MAINTENANCE_CMD = 0x60,
    STEPPER_SPECS_CMD = 0x70,
  };

  enum class MaintenanceCommandOptions : uint8_t
  {
    SET_CURRENT_MULTI_TURN_POS_ZERO_TO_ROM_CMD = 0x00,
    MOTOR_STOP_CMD = 0x01,
    MOTOR_SHUTDOWN_CMD = 0x02,
    CLEAR_ERRORS_CMD = 0x03,
  };

  enum class StatusCommands : uint8_t
  {
    MOTOR_STATE = 0x40,
    MOTOR_STATUS = 0x50,
  };

  enum class MotorStatus : uint8_t
  {
    UNDEFINED                = 0,
    IDLE                     = 1,
    STARTUP_SEQUENCE         = 2,

    ERROR_INVALID_REQUEST    = 3,
    ERROR_STEPPER_DISARMED   = 4,
    ERROR_STEPPER_FAILED     = 5,
    ERROR_CONTROLLER_FAILED  = 6,
    ERROR_ESTOP_REQUESTED    = 7,
    ERROR_UNKNOWN_POSITION   = 8,

    POSITION_CONTROL         = 9,
    VELOCITY_CONTROL         = 10,
    STEPPER_STOPPED          = 11
  };

  enum class ValidateRequest : uint8_t
  {
    INVALID = 0,
    VALID = 1,
  };

  static constexpr std::array<StatusCommands, 2> kStatusCommands = {
    StatusCommands::MOTOR_STATE,
    StatusCommands::MOTOR_STATUS,
  };

  inline DecodedCommand unpack_command_full(int32_t counts_in, int64_t payload_in)
  {
    const uint32_t counts = static_cast<uint32_t>(counts_in);
    const uint64_t payload = static_cast<uint64_t>(payload_in);

    const uint8_t u8_count = static_cast<uint8_t>((counts >> 16) & 0xFF);
    const uint8_t i16_count = static_cast<uint8_t>((counts >> 8) & 0xFF);
    const uint8_t i32_count = static_cast<uint8_t>(counts & 0xFF);

    int bit_offset = 64;
    auto pop_bits = [&](int bits) -> uint64_t {
      bit_offset -= bits;
      const uint64_t mask = (bits == 64) ? std::numeric_limits<uint64_t>::max() :
        ((1ULL << bits) - 1ULL);
      return (payload >> bit_offset) & mask;
    };

    DecodedCommand result{};
    result.command_id = static_cast<uint8_t>(pop_bits(8));
    for (uint8_t i = 0; i < u8_count; ++i) {
      result.u8_data.push_back(static_cast<uint8_t>(pop_bits(8)));
    }
    for (uint8_t i = 0; i < i16_count; ++i) {
      result.i16_data.push_back(static_cast<int16_t>(static_cast<uint16_t>(pop_bits(16))));
    }
    for (uint8_t i = 0; i < i32_count; ++i) {
      result.i32_data.push_back(static_cast<int32_t>(static_cast<uint32_t>(pop_bits(32))));
    }
    return result;
  }

  template<typename JointT>
  inline void pack_decoded_maintenance_frame(
    JointT & joint, const DecodedCommand & decoded_maintenance_cmd)
  {
    joint.decoded_maintenance_frame.clear();
    joint.decoded_maintenance_frame.push_back(decoded_maintenance_cmd.command_id);
    joint.decoded_maintenance_frame.insert(
      joint.decoded_maintenance_frame.end(),
      decoded_maintenance_cmd.u8_data.begin(),
      decoded_maintenance_cmd.u8_data.end());
  }
};

}  // namespace stepper_ros2_control

#endif  // STEPPER_HARDWARE_INTERFACE_HPP_

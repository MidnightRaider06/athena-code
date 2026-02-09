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

#ifndef KILLSWITCH_ROS2_CONTROL__KILLSWITCH_HARDWARE_INTERFACE_HPP_
#define KILLSWITCH_ROS2_CONTROL__KILLSWITCH_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <array>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "umdloop_can_library/SocketCanBus.hpp"
#include "umdloop_can_library/CanFrame.hpp"

namespace killswitch_ros2_control
{

/**
 * @brief Hardware interface for CAN-controlled killswitch via ros2_control
 * 
 * This interface sends kill commands to the voltage monitoring board via CAN.
 * 
 * CAN Protocol (ID: 0x100):
 * - Kill All Power:    DATA[0] = 0x01
 * - Kill Main Power:   DATA[0] = 0x03
 * - Kill Jetson Power: DATA[0] = 0x05
 * 
 * State Interfaces (read by controllers):
 * - kill_all_sent: 1.0 if kill all command was sent
 * - kill_main_sent: 1.0 if kill main command was sent
 * - kill_jetson_sent: 1.0 if kill jetson command was sent
 * - is_connected: Is CAN connected (0.0 or 1.0)
 * 
 * Command Interfaces (written by controllers):
 * - kill_all: Set to 1.0 to kill all power
 * - kill_main: Set to 1.0 to kill main power
 * - kill_jetson: Set to 1.0 to kill jetson power
 * 
 * Hardware Parameters (from URDF):
 * - can_interface: CAN interface name (default: "can0")
 * - can_id: CAN ID for killswitch commands (default: 0x100)
 */
class KillswitchHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KillswitchHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // CAN message handler
  void onCanMessage(const CANLib::CanFrame& frame);

  // Configuration parameters
  std::string can_interface_;
  uint32_t can_id_;

  // CAN bus
  CANLib::SocketCanBus canBus_;
  CANLib::CanFrame can_tx_frame_;
  bool can_connected_;

  // State variables (track what was sent)
  double kill_all_sent_;
  double kill_main_sent_;
  double kill_jetson_sent_;
  double is_connected_;

  // Command variables
  double cmd_kill_all_;
  double cmd_kill_main_;
  double cmd_kill_jetson_;

  // CAN command bytes
  static constexpr uint8_t CMD_KILL_ALL = 0x01;
  static constexpr uint8_t CMD_KILL_MAIN = 0x03;
  static constexpr uint8_t CMD_KILL_JETSON = 0x05;
};

}  // namespace killswitch_ros2_control

#endif  // KILLSWITCH_ROS2_CONTROL__KILLSWITCH_HARDWARE_INTERFACE_HPP_

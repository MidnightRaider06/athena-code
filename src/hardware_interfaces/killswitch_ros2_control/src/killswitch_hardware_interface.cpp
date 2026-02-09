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

#include "killswitch_ros2_control/killswitch_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace killswitch_ros2_control
{

hardware_interface::CallbackReturn KillswitchHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != 
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize state variables
  kill_all_sent_ = 0.0;
  kill_main_sent_ = 0.0;
  kill_jetson_sent_ = 0.0;
  is_connected_ = 0.0;

  // Initialize command variables
  cmd_kill_all_ = 0.0;
  cmd_kill_main_ = 0.0;
  cmd_kill_jetson_ = 0.0;

  // Parse hardware parameters
  if (info_.hardware_parameters.count("can_interface")) {
    can_interface_ = info_.hardware_parameters.at("can_interface");
  } else {
    can_interface_ = "can0";
  }

  if (info_.hardware_parameters.count("can_id")) {
    // Parse hex string (e.g., "0x100") - base 0 auto-detects hex/decimal
    can_id_ = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0));
  } else {
    can_id_ = 0x100;  // Default killswitch CAN ID
  }

  can_connected_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Initialized killswitch on CAN interface %s with ID 0x%X", 
    can_interface_.c_str(), can_id_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KillswitchHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Configuring killswitch hardware...");

  // Open CAN bus
  if (!canBus_.open(can_interface_, 
      std::bind(&KillswitchHardwareInterface::onCanMessage, this, std::placeholders::_1))) 
  {
    RCLCPP_WARN(
      rclcpp::get_logger("KillswitchHardwareInterface"),
      "Failed to open CAN interface %s - running in SIMULATION mode", 
      can_interface_.c_str());
    can_connected_ = false;
  } else {
    can_connected_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("KillswitchHardwareInterface"),
      "Successfully opened CAN interface %s", can_interface_.c_str());
  }

  is_connected_ = can_connected_ ? 1.0 : 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Killswitch hardware configured (%s)", can_connected_ ? "CAN MODE" : "SIMULATION");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void KillswitchHardwareInterface::onCanMessage(const CANLib::CanFrame& frame)
{
  // Handle incoming CAN messages from voltage monitoring board if needed
  (void)frame;  // Currently not expecting responses
}

std::vector<hardware_interface::StateInterface> 
KillswitchHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Use the gpio name from URDF
  const std::string& name = info_.gpios[0].name;

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "kill_all_sent", &kill_all_sent_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "kill_main_sent", &kill_main_sent_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "kill_jetson_sent", &kill_jetson_sent_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "is_connected", &is_connected_));

  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Exported %zu state interfaces", state_interfaces.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> 
KillswitchHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Use the gpio name from URDF
  const std::string& name = info_.gpios[0].name;

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(name, "kill_all", &cmd_kill_all_));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(name, "kill_main", &cmd_kill_main_));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(name, "kill_jetson", &cmd_kill_jetson_));

  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Exported %zu command interfaces", command_interfaces.size());

  return command_interfaces;
}

hardware_interface::CallbackReturn KillswitchHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Activating killswitch hardware...");

  // Reset commands to safe state
  cmd_kill_all_ = 0.0;
  cmd_kill_main_ = 0.0;
  cmd_kill_jetson_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KillswitchHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Deactivating killswitch hardware...");

  // Note: We intentionally do NOT send kill commands on deactivate
  // Killswitch should only respond to explicit commands

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KillswitchHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Cleaning up killswitch hardware...");

  if (can_connected_) {
    canBus_.close();
  }

  can_connected_ = false;
  is_connected_ = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Killswitch hardware cleanup complete");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KillswitchHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    rclcpp::get_logger("KillswitchHardwareInterface"),
    "Shutting down killswitch hardware...");

  return on_cleanup(previous_state);
}

hardware_interface::return_type KillswitchHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // CAN messages are handled asynchronously via callback
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KillswitchHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // Check kill_all command (rising edge detection)
  if (cmd_kill_all_ > 0.5 && kill_all_sent_ < 0.5) {
    if (can_connected_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = can_id_;
      can_tx_frame_.dlc = 1;
      can_tx_frame_.data[0] = CMD_KILL_ALL;
      canBus_.send(can_tx_frame_);
    }
    
    kill_all_sent_ = 1.0;
    RCLCPP_ERROR(
      rclcpp::get_logger("KillswitchHardwareInterface"),
      "KILL ALL POWER COMMAND SENT%s", can_connected_ ? "" : " (simulated)");
  } else if (cmd_kill_all_ < 0.5) {
    kill_all_sent_ = 0.0;  // Reset when command goes low
  }

  // Check kill_main command (rising edge detection)
  if (cmd_kill_main_ > 0.5 && kill_main_sent_ < 0.5) {
    if (can_connected_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = can_id_;
      can_tx_frame_.dlc = 1;
      can_tx_frame_.data[0] = CMD_KILL_MAIN;
      canBus_.send(can_tx_frame_);
    }
    
    kill_main_sent_ = 1.0;
    RCLCPP_WARN(
      rclcpp::get_logger("KillswitchHardwareInterface"),
      "KILL MAIN POWER COMMAND SENT%s", can_connected_ ? "" : " (simulated)");
  } else if (cmd_kill_main_ < 0.5) {
    kill_main_sent_ = 0.0;  // Reset when command goes low
  }

  // Check kill_jetson command (rising edge detection)
  if (cmd_kill_jetson_ > 0.5 && kill_jetson_sent_ < 0.5) {
    if (can_connected_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = can_id_;
      can_tx_frame_.dlc = 1;
      can_tx_frame_.data[0] = CMD_KILL_JETSON;
      canBus_.send(can_tx_frame_);
    }
    
    kill_jetson_sent_ = 1.0;
    RCLCPP_WARN(
      rclcpp::get_logger("KillswitchHardwareInterface"),
      "KILL JETSON POWER COMMAND SENT%s", can_connected_ ? "" : " (simulated)");
  } else if (cmd_kill_jetson_ < 0.5) {
    kill_jetson_sent_ = 0.0;  // Reset when command goes low
  }

  return hardware_interface::return_type::OK;
}

}  // namespace killswitch_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  killswitch_ros2_control::KillswitchHardwareInterface,
  hardware_interface::SystemInterface)

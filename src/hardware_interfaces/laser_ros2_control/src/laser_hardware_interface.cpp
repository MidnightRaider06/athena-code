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

#include "laser_ros2_control/laser_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace laser_ros2_control
{

hardware_interface::CallbackReturn LaserHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != 
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize state variables
  laser_state_ = 0.0;      // OFF
  temperature_ = 0.0;
  is_connected_ = 0.0;

  // Initialize command variables
  laser_command_ = 0.0;    // OFF

  // Parse hardware parameters
  if (info_.hardware_parameters.count("can_interface")) {
    can_interface_ = info_.hardware_parameters.at("can_interface");
  } else {
    can_interface_ = "can0";
  }

  if (info_.hardware_parameters.count("can_id")) {
    // Parse hex string (e.g., "0x130") - base 0 auto-detects hex/decimal
    can_id_ = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0));
  } else {
    can_id_ = 0x130;  // Default laser CAN ID
  }

  can_connected_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Initialized laser on CAN interface %s with ID 0x%X", 
    can_interface_.c_str(), can_id_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Configuring laser hardware...");

  // Open CAN bus
  if (!canBus_.open(can_interface_, 
      std::bind(&LaserHardwareInterface::onCanMessage, this, std::placeholders::_1))) 
  {
    RCLCPP_WARN(
      rclcpp::get_logger("LaserHardwareInterface"),
      "Failed to open CAN interface %s - running in SIMULATION mode", 
      can_interface_.c_str());
    can_connected_ = false;
  } else {
    can_connected_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("LaserHardwareInterface"),
      "Successfully opened CAN interface %s", can_interface_.c_str());
  }

  is_connected_ = can_connected_ ? 1.0 : 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Laser hardware configured (%s)", can_connected_ ? "CAN MODE" : "SIMULATION");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void LaserHardwareInterface::onCanMessage(const CANLib::CanFrame& frame)
{
  // Handle incoming CAN messages (e.g., temperature readings)
  if (frame.id == can_id_) {
    // Parse response based on command byte
    if (frame.dlc > 0 && frame.data[0] == CMD_READ_TEMP) {
      // Temperature response - parse if available
      if (frame.dlc >= 2) {
        temperature_ = static_cast<double>(frame.data[1]);
      }
    }
  }
}

std::vector<hardware_interface::StateInterface> 
LaserHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Use the gpio name from URDF
  const std::string& name = info_.gpios[0].name;

  // Laser state (ON/OFF)
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "laser_state", &laser_state_));

  // Temperature
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "temperature", &temperature_));

  // Connection status
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "is_connected", &is_connected_));

  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Exported %zu state interfaces", state_interfaces.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> 
LaserHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Use the gpio name from URDF
  const std::string& name = info_.gpios[0].name;

  // Laser command (ON/OFF)
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(name, "laser_command", &laser_command_));

  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Exported %zu command interfaces", command_interfaces.size());

  return command_interfaces;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Activating laser hardware...");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Deactivating laser hardware...");

  // Safety: Turn laser OFF on deactivation
  if (can_connected_) {
    can_tx_frame_ = CANLib::CanFrame();
    can_tx_frame_.id = can_id_;
    can_tx_frame_.dlc = 1;
    can_tx_frame_.data[0] = CMD_LASER_OFF;
    canBus_.send(can_tx_frame_);
    
    RCLCPP_INFO(
      rclcpp::get_logger("LaserHardwareInterface"),
      "Laser turned OFF (safety)");
  }

  laser_state_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Cleaning up laser hardware...");

  // Ensure laser is OFF before cleanup
  if (can_connected_) {
    can_tx_frame_ = CANLib::CanFrame();
    can_tx_frame_.id = can_id_;
    can_tx_frame_.dlc = 1;
    can_tx_frame_.data[0] = CMD_LASER_OFF;
    canBus_.send(can_tx_frame_);
    
    canBus_.close();
  }

  can_connected_ = false;
  is_connected_ = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Laser hardware cleanup complete");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Shutting down laser hardware...");

  return on_cleanup(previous_state);
}

hardware_interface::return_type LaserHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // CAN messages are handled asynchronously via callback
  // State is updated in onCanMessage()
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LaserHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // Check if laser command changed
  bool commanded_on = (laser_command_ > 0.5);
  bool currently_on = (laser_state_ > 0.5);

  if (commanded_on != currently_on) {
    if (can_connected_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = can_id_;
      can_tx_frame_.dlc = 1;
      can_tx_frame_.data[0] = commanded_on ? CMD_LASER_ON : CMD_LASER_OFF;
      canBus_.send(can_tx_frame_);
    }

    laser_state_ = commanded_on ? 1.0 : 0.0;
    
    RCLCPP_INFO(
      rclcpp::get_logger("LaserHardwareInterface"),
      "Laser turned %s%s", commanded_on ? "ON" : "OFF",
      can_connected_ ? "" : " (simulated)");
  }

  return hardware_interface::return_type::OK;
}

}  // namespace laser_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  laser_ros2_control::LaserHardwareInterface,
  hardware_interface::SystemInterface)

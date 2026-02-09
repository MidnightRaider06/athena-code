#include "talon_ros2_control/talon_hardware_interface.hpp"

#include <netdb.h>
#include <sys/socket.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <cstring>
#include <sstream>
#include <string>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "talon_ros2_control/talon_control.h"

namespace talon_ros2_control
{
TALONHardwareInterface::TALONHardwareInterface() {
}

void TALONHardwareInterface::logger_function(){

  // Prevents breaking the logger
  if (num_joints == 0) return;

  // Building Message
  std::string log_msg = "\033[2J\033[H \nTALON Logger";
  std::string control_mode = "";
  std::ostringstream oss;
  std::string status;
  
  // HWI Specific
  oss << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface
      << " | HWI Update Rate: " << update_rate
      << " | Logger Update Rate: " << logger_rate << "\n"
      << "Elapsed Time since first update: " << elapsed_time << "\n"
      << "\n--- Joint Specific ---";

  for (int i = 0; i < num_joints; i++) {
    if(static_cast<int>(control_level_[i]) == 1) {
      control_mode = "POSITION";
    }
    else if(static_cast<int>(control_level_[i]) == 2) {
      control_mode = "VELOCITY";
    }
    else {
      control_mode = "UNDEFINED";
    }

    oss << "\nJOINT: " << info_.joints[i].name << "\n"
        << "Parameters: Node ID: 0x" << std::hex << std::uppercase << joint_node_ids[i] << "\n"
        << "-- Commands --\n"
        << "Control Mode: " << control_mode << "\n"
        << "Joint Command Position: " << joint_command_position_[i] << "\n"
        << "Joint Command Velocity: " << joint_command_velocity_[i] << "\n"
        << "-- State --\n"
        << "Joint Position: " << joint_state_position_[i] << "\n"
        << "Joint Velocity: " << joint_state_velocity_[i] << "\n";
  }

  log_msg += oss.str();
  RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), log_msg.c_str());
}

hardware_interface::CallbackReturn TALONHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info) // Info stores all parameters in xacro file
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // -- Parameters --
  can_interface = info_.hardware_parameters.at("can_interface");
  update_rate = std::stoi(info_.hardware_parameters.at("update_rate"));
  logger_rate = std::stoi(info_.hardware_parameters.at("logger_rate"));
  logger_state = std::stoi(info_.hardware_parameters.at("logger_state"));

  for (auto& joint : info_.joints) {
    joint_node_ids.push_back(std::stoi(joint.parameters.at("node_id")));

    std::string joint_type = joint.parameters.at("joint_type");

    // Revolute or Prismatic
    if (joint_type == "revolute") {
      joint_type_.push_back(joint_type_t::REVOLUTE);
      max_disp.push_back(std::nan("")); // Not used for revolute joints
    } else if (joint_type == "prismatic" && joint.parameters.count("max_disp")) {
      joint_type_.push_back(joint_type_t::PRISMATIC);
      max_disp.push_back(std::abs(std::stod(joint.parameters.at("max_disp"))));
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("TALONHardwareInterface"), "Invalid joint_type parameter for joint %s. Must be 'revolute' or 'prismatic'. If prismatic, it must have a 'max_disp' parameter.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  num_joints = static_cast<int>(info_.joints.size());

  // Initializes command and state interface values
  joint_state_position_.assign(num_joints, 0);
  joint_state_velocity_.assign(num_joints, 0);

  joint_command_position_.assign(num_joints, 0);
  joint_command_velocity_.assign(num_joints, 0);

  control_level_.resize(num_joints, integration_level_t::POSITION);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TALONHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  // Reuse cleanup logic which shuts down the motor and then deinitializes shared pointers.
  // Need this in case on_cleanup never gets called
  return this->on_cleanup(previous_state);
}

std::vector<hardware_interface::StateInterface> TALONHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for(int i = 0; i < num_joints; i++){
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_state_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_state_velocity_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TALONHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for(int i = 0; i < num_joints; i++){
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_command_position_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_command_velocity_[i]));
  }

  return command_interfaces;
}

/*
* Setup communication with motor (init configs, send first CAN message)
*/
hardware_interface::CallbackReturn TALONHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "Beginning configure.");

    for(int i = 0; i < num_joints; i++){
      TalonSRX *motor = new TalonSRX(joint_node_ids[i], can_interface);
      talon_motors.push_back(motor);
      RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "talon_motor initialized.");
      initMotor(motor);
    }

    RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "Configure complete.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TALONHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/) {

    for(int i = 0; i < num_joints; i++){
      stopMotor(talon_motors[i], 50);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

void TALONHardwareInterface::enable_system_thread() {
  while (is_running.load()) {
    ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

/*
* Activate system
*/
hardware_interface::CallbackReturn TALONHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "Activating ...please wait...");

  is_running.store(true);
  worker = std::thread(&TALONHardwareInterface::enable_system_thread, this);

  // setPositionFromDisplacement(talon_motor, 0.0, 50); // dangerous atm since this is an incremental encoder, dont want this to crush itself

  RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/*
* Deactivate system
*/
hardware_interface::CallbackReturn TALONHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "Deactivating ...please wait...");

  for(int i = 0; i < num_joints; i++){
    setVelocityFromLinearVelocity(talon_motors[i], 0.0, 50);
  }

  is_running.store(false);
  if (worker.joinable()) {
    worker.join();
  }

  // setPositionFromDisplacement(talon_motor, 0.0, 50); // dangerous atm since this is an incremental encoder, dont want this to crush itself

  RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TALONHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for(int i = 0; i < num_joints; i++){
    joint_state_position_[i] = getPositionDistance(talon_motors[i]); // meters
    joint_state_velocity_[i] = getClawVelocity(talon_motors[i]); // m/s
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type talon_ros2_control::TALONHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  elapsed_update_time+=period.seconds();
  double update_period = 1.0/update_rate;
  int32_t joint_angle = 0;
  int32_t joint_velocity = 0;

  elapsed_time+=period.seconds();
  
  // Logger update
  elapsed_logger_time+=period.seconds();
  double logging_period = 1.0/logger_rate;
  if(elapsed_logger_time > logging_period){
    elapsed_logger_time = 0.0;
    if (logger_state == 1) {
      logger_function();
    }
  }

  // HWI can only go as fast as the controller manager. To limit frequency of bus messages,
  // keep track of time passed over iterations of this function and if it exceeds the 
  // desired frequency of the HWI, skip message
  if(elapsed_update_time > update_period){
    elapsed_update_time = 0.0;

    // Motor Command for each joint at given frequency
    for(int i = 0; i < num_joints; i++){
      if(control_level_[i] == integration_level_t::POSITION && joint_type_[i] == joint_type_t::PRISMATIC && std::isfinite(joint_command_position_[i])) {

        // TO DO: implement joint limits so i dont gotta do this
        joint_command_position_[i] = std::clamp(joint_command_position_[i], 0.0, max_disp[i]);
        // COMMAND PRISMATIC POSITION
        setPositionFromDisplacement(talon_motors[i], joint_command_position_[i], 50);

      } else if(control_level_[i] == integration_level_t::VELOCITY && joint_type_[i] == joint_type_t::PRISMATIC && std::isfinite(joint_command_velocity_[i])) {

        // COMMAND PRISMATIC VELOCITY
        setVelocityFromLinearVelocity(talon_motors[i], joint_command_velocity_[i], 50);

      } else if(control_level_[i] == integration_level_t::POSITION && joint_type_[i] == joint_type_t::REVOLUTE && std::isfinite(joint_command_position_[i])) {

        // COMMAND REVOLUTE POSITION
        setPositionFromJointCommand(talon_motors[i], joint_command_position_[i], 50);

      } else if(control_level_[i] == integration_level_t::VELOCITY && joint_type_[i] == joint_type_t::REVOLUTE && std::isfinite(joint_command_velocity_[i])) {

        // COMMAND REVOLUTE VELOCITY
        setVelocityFromAngularVelocity(talon_motors[i], joint_command_velocity_[i], 50);

      }
      else {
        // RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "Joint command value not found or undefined command state");
      }
    }
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TALONHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string>& start_interfaces,
  const std::vector<std::string>& stop_interfaces)
{
  // Debug: print incoming requests
  std::ostringstream ss;
  ss << "perform_command_mode_switch called. start_interfaces: [";
  for (auto &s : start_interfaces) ss << s << ",";
  ss << "] stop_interfaces: [";
  for (auto &s : stop_interfaces) ss << s << ",";
  ss << "]";
  RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), ss.str().c_str());

  // For each joint, decide its new control mode based on start/stop interfaces.
  // We allow partial starts/stops: only affected joints are switched.
  std::vector<integration_level_t> requested_modes(num_joints, integration_level_t::UNDEFINED);

  // Process stop interfaces first: mark those joints as UNDEFINED
  for (const auto &ifname : stop_interfaces) {
    for (int i = 0; i < num_joints; ++i) {
      const std::string pos_if = info_.joints[i].name + "/" + std::string(hardware_interface::HW_IF_POSITION);
      const std::string vel_if = info_.joints[i].name + "/" + std::string(hardware_interface::HW_IF_VELOCITY);
      if (ifname == pos_if || ifname == vel_if || ifname.find(info_.joints[i].name) != std::string::npos) {
        requested_modes[i] = integration_level_t::UNDEFINED;
      }
    }
  }

  // Process start interfaces: set POSITION or VELOCITY per joint
  for (const auto &ifname : start_interfaces) {
    for (int i = 0; i < num_joints; ++i) {
      const std::string pos_if = info_.joints[i].name + "/" + std::string(hardware_interface::HW_IF_POSITION);
      const std::string vel_if = info_.joints[i].name + "/" + std::string(hardware_interface::HW_IF_VELOCITY);
      if (ifname == pos_if) {
        requested_modes[i] = integration_level_t::POSITION;
      } else if (ifname == vel_if) {
        requested_modes[i] = integration_level_t::VELOCITY;
      }
    }
  }

  // Now apply the requested_modes to control_level_.
  // For any joint with UNDEFINED in requested_modes, we only change it if it was explicitly stopped.
  for (int i = 0; i < num_joints; ++i) {
    if (requested_modes[i] == integration_level_t::UNDEFINED) {
      // if stop requested, set to UNDEFINED; otherwise leave existing mode
      // (we only set to UNDEFINED if this joint was mentioned in stop_interfaces)
      bool was_stopped = false;
      for (const auto &ifname : stop_interfaces) {
        if (ifname.find(info_.joints[i].name) != std::string::npos) {
          was_stopped = true;
          break;
        }
      }
      if (was_stopped) {
        control_level_[i] = integration_level_t::UNDEFINED;
        joint_command_velocity_[i] = 0;
        // optional: reset position cmd to current state to be safe
        joint_command_position_[i] = joint_state_position_[i];
        RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"),
          "Joint %s: stopped -> set UNDEFINED", info_.joints[i].name.c_str());
      }
      // else, leave control_level_ as-is
    } else {
      // Set the mode requested
      control_level_[i] = requested_modes[i];
      // If switching to velocity, optionally set command velocity to current state to avoid jumps
      if (requested_modes[i] == integration_level_t::VELOCITY) {
        // joint_command_velocity_[i] = joint_state_velocity_[i];
        joint_command_velocity_[i] = 0;
        RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"),
          "Joint %s: switched to VELOCITY (cmd vel initialized from state: %f)",
          info_.joints[i].name.c_str(), joint_command_velocity_[i]);
      } else if (requested_modes[i] == integration_level_t::POSITION) {
        joint_command_position_[i] = joint_state_position_[i];
        RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"),
          "Joint %s: switched to POSITION (cmd pos initialized from state: %f)",
          info_.joints[i].name.c_str(), joint_command_position_[i]);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace talon_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  talon_ros2_control::TALONHardwareInterface, hardware_interface::SystemInterface)

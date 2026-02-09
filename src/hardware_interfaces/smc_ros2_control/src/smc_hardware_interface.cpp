#include "smc_ros2_control/smc_hardware_interface.hpp"

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
#include <cmath>
#include <algorithm>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace smc_ros2_control
{

double SMCHardwareInterface::calculate_joint_position_from_motor_position(double motor_position, int gear_ratio){
  // Converts from 0.01 deg to deg to radians/s
  return (motor_position * 0.01 * (M_PI/180.0))/gear_ratio;
}

double SMCHardwareInterface::calculate_joint_velocity_from_motor_velocity(double motor_velocity, int gear_ratio){
  // Converts from dps to radians/s
  return (motor_velocity * (M_PI/180.0))/gear_ratio;
}

int32_t SMCHardwareInterface::calculate_motor_position_from_desired_joint_position(double joint_position, int gear_ratio){
  // radians -> deg -> 0.01 deg
  return static_cast<int32_t>(std::round((joint_position*(180/M_PI)*100)*gear_ratio));
}

int32_t SMCHardwareInterface::calculate_motor_velocity_from_desired_joint_velocity(double joint_velocity, int gear_ratio){
  // radians/s -> deg/s -> 0.01 deg/s
  return static_cast<int32_t>(std::round((joint_velocity*(180/M_PI)*100)*gear_ratio));
}

void SMCHardwareInterface::send_command(int can_id, int cmd_id){
  CANLib::CanFrame frame;
  frame.id  = can_id;
  frame.dlc = 8;
  frame.data.fill(0);
  frame.data[0] = cmd_id;
  canBus.send(frame);
}

void SMCHardwareInterface::logger_function(){

  // Prevents breaking the logger
  if (num_joints == 0) return;

  // Building Message
  std::string log_msg = "\033[2J\033[H \nSMC Logger";
  std::string control_mode = "";
  
  // HWI Specific
  std::ostringstream oss;

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
        << "Parameters: CAN ID: 0x" << std::hex << std::uppercase << joint_node_ids[i]
        << " | Gear Ratio: " << joint_gear_ratios[i] << "\n"
        << "-- Commands --\n"
        << "Control Mode: " << control_mode << "\n"
        << "Motor Position: " << motor_position[i]
        << " | Joint Command Position: " << joint_command_position_[i] << "\n"
        << "Motor Velocity: " << motor_velocity[i]
        << " | Joint Command Velocity: " << joint_command_velocity_[i] << "\n"
        << "-- State --\n"
        << "Joint Position: " << joint_state_position_[i]
        << " | Joint Velocity: " << joint_state_velocity_[i] << "\n";
  }

  log_msg += oss.str();
  RCLCPP_INFO(rclcpp::get_logger("SMCHardwareInterface"), log_msg.c_str());
}


hardware_interface::CallbackReturn SMCHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info) // Info stores all parameters in xacro file
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Setting Parameters
  for (auto& joint : info_.joints) {
    int gear_ratio = std::abs(std::stoi(joint.parameters.at("gear_ratio")));
    joint_node_ids.push_back(std::clamp(std::stoi(joint.parameters.at("node_id"), nullptr, 0), 0x141, 0x160));
    joint_gear_ratios.push_back(gear_ratio);
    operating_velocity = std::clamp(std::stoi(joint.parameters.at("operating_velocity")), 0, 65*gear_ratio);
  }

  num_joints = static_cast<int>(info_.joints.size());
  update_rate = std::stoi(info_.hardware_parameters.at("update_rate"));
  logger_rate = std::stoi(info_.hardware_parameters.at("logger_rate"));
  logger_state = std::stoi(info_.hardware_parameters.at("logger_state"));
  can_interface = info_.hardware_parameters.at("can_interface");

  elapsed_update_time = 0.0;
  elapsed_time = 0.0;
  elapsed_logger_time = 0.0;
  
  // Initializes command and state interface values
  joint_state_position_.assign(num_joints, std::numeric_limits<double>::quiet_NaN());
  joint_state_velocity_.assign(num_joints, 0);

  joint_command_position_.assign(num_joints, std::numeric_limits<double>::quiet_NaN());
  joint_command_velocity_.assign(num_joints, 0);

  encoder_position.assign(num_joints, 0.0);
  motor_position.assign(num_joints, 0.0);
  motor_velocity.assign(num_joints, 0.0);

  control_level_.resize(num_joints, integration_level_t::POSITION);

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn SMCHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  // Reuse cleanup logic which shuts down the motor and then deinitializes shared pointers.
  // Need this in case on_cleanup never gets called
  return this->on_cleanup(previous_state);
}


std::vector<hardware_interface::StateInterface> SMCHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Each SMC motor corresponds to a different joint.
  for(int i = 0; i < num_joints; i++){   
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_state_position_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_state_velocity_[i]));
  }
  
  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface>
SMCHardwareInterface::export_command_interfaces()
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

hardware_interface::CallbackReturn SMCHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!canBus.open(can_interface, std::bind(&SMCHardwareInterface::on_can_message, this, std::placeholders::_1))) {
    RCLCPP_INFO(rclcpp::get_logger("SMCHardwareInterface"), "Failed to open CAN interface");
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

void SMCHardwareInterface::on_can_message(const CANLib::CanFrame& frame) {
  can_rx_frame_ = frame; // Store the received frame for processing in read()

  std::string result;

  int data[8] = {0x00};

  for(int i = 0; i < num_joints; i++){  
    if(can_rx_frame_.id == joint_node_ids[i] && 
      (can_rx_frame_.data[0] == SPEED_CONTROL_CMD  || 
       can_rx_frame_.data[0] == ABSOLUTE_POS_CONTROL_CMD  || 
       can_rx_frame_.data[0] == MOTOR_STATUS_2_CMD) &&
       can_rx_frame_.data[1] != 0x00){

      // DECODING CAN MESSAGE FOR VELOCITY
      data[1] = can_rx_frame_.data[1]; // Motor Temperature
      data[2] = can_rx_frame_.data[2]; // Torque low byte
      data[3] = can_rx_frame_.data[3]; // Torque high byte
      data[4] = can_rx_frame_.data[4]; // speed low byte
      data[5] = can_rx_frame_.data[5]; // speed high byte
      data[6] = can_rx_frame_.data[6]; // encoder position low byte
      data[7] = can_rx_frame_.data[7]; // encoder position high byte

      // ENCODER POSITION 
      // uint16 -> int16 -> double (for calcs)
      encoder_position[i] = static_cast<double>(static_cast<int16_t>((data[7] << 8) | data[6]));

      // SPEED
      // uint16 -> int16 -> double (for calcs)
      motor_velocity[i] = static_cast<double>(static_cast<int16_t>((data[5] << 8) | data[4]));
    }
    else if(can_rx_frame_.id == joint_node_ids[i] && can_rx_frame_.data[0] == 0x92){
      data[0] = 0x92;
      data[1] = can_rx_frame_.data[1]; // Multi-turn low byte
      data[2] = can_rx_frame_.data[2];
      data[3] = can_rx_frame_.data[3];
      data[4] = can_rx_frame_.data[4];
      data[5] = can_rx_frame_.data[5]; 
      data[6] = can_rx_frame_.data[6];
      data[7] = can_rx_frame_.data[7]; // Multi-turn high byte

      // POSITION
      // In this case, we have 56 bits to store in 64, so we must have a sign extension
      // uint64 -> int64 (to do sign extension) -> sign extension (left shift 8, then arithmetic right shift 8) --> double (for calcs)
      uint64_t motor_position_raw = (static_cast<uint64_t>(data[7]) << 48)   | 
                                    (static_cast<uint64_t>(data[6]) << 40)   | 
                                    (static_cast<uint64_t>(data[5]) << 32)   |
                                    (static_cast<uint64_t>(data[4]) << 24)   | 
                                    (static_cast<uint64_t>(data[3]) << 16)   | 
                                    (static_cast<uint64_t>(data[2]) << 8)    | 
                                     static_cast<uint64_t>(data[1]);

      motor_position[i] = static_cast<double>((static_cast<int64_t>(motor_position_raw) << 8) >> 8);      
    }
    else{
      if(logger_state == 1) {
        // RCLCPP_INFO(rclcpp::get_logger("SMCHardwareInterface"), "Reply not heard.");
      }
    }    
  }

}

hardware_interface::CallbackReturn SMCHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // If cleanup occurs before shutdown, this is the last opportunity to shutdown motor since pointers must be deleted here
  for(int i = 0; i < num_joints; i++){
    // Motor Off (Shutdown) Command
    send_command(joint_node_ids[i], MOTOR_SHUTDOWN_CMD);
  }

  canBus.close();

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn SMCHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SMCHardwareInterface"), "Activating ...please wait...");

  for(int i = 0; i < num_joints; i++){
    send_command(joint_node_ids[i], MOTOR_RUNNING_CMD);
  }

  // Sets initial command to joint state
  joint_command_position_ = joint_state_position_;

  RCLCPP_INFO(rclcpp::get_logger("SMCHardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn SMCHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SMCHardwareInterface"), "Deactivating ...please wait...");

  for(int i = 0; i < num_joints; i++){
    send_command(joint_node_ids[i], MOTOR_STOP_CMD);
  }

  RCLCPP_INFO(rclcpp::get_logger("SMCHardwareInterface"), "Successfully deactivated all SMC motors!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type SMCHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  //CALCULATING JOINT STATE
  for(int i = 0; i < num_joints; i++){  
    joint_state_velocity_[i] = calculate_joint_velocity_from_motor_velocity(motor_velocity[i], joint_gear_ratios[i]);
    joint_state_position_[i] = calculate_joint_position_from_motor_position(motor_position[i], joint_gear_ratios[i]);
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type smc_ros2_control::SMCHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  elapsed_update_time+=period.seconds();
  double update_period = 1.0/update_rate;
  int data[8] = {0x00};
  int32_t joint_angle = 0;
  int16_t operating_velocity = 200;
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
    for(int i = 0; i < num_joints; i++) {
      can_tx_frame_ = CANLib::CanFrame(); // Must reinstantiate else data from past iteration gets repeated
      can_tx_frame_.id = joint_node_ids[i];
      can_tx_frame_.dlc = 8;

      if(control_level_[i] == integration_level_t::POSITION && std::isfinite(joint_command_position_[i])) {
        
        // CALCULATE DESIRED JOINT ANGLE
        joint_angle = calculate_motor_position_from_desired_joint_position(joint_command_position_[i], joint_gear_ratios[i]);
        
        // ENCODING CAN MESSAGE
        data[0] = ABSOLUTE_POS_CONTROL_CMD; 
        data[1] = 0x00;
        data[2] = operating_velocity & 0xFF;
        data[3] = (operating_velocity >> 8) & 0xFF;
        data[4] = joint_angle & 0xFF;
        data[5] = (joint_angle >> 8) & 0xFF;
        data[6] = (joint_angle >> 16) & 0xFF;
        data[7] = (joint_angle >> 24) & 0xFF;
      }
      else if(control_level_[i] == integration_level_t::VELOCITY && std::isfinite(joint_command_velocity_[i])) {

        // CALCULATE DESIRED JOINT VELOCITY
        joint_velocity = calculate_motor_velocity_from_desired_joint_velocity(joint_command_velocity_[i], joint_gear_ratios[i]);
   
        // ENCODING CAN MESSAGE
        data[0] = SPEED_CONTROL_CMD;
        data[1] = 0x00;
        data[2] = 0x00;
        data[3] = 0x00;  
        data[4] = joint_velocity & 0xFF;
        data[5] = (joint_velocity >> 8) & 0xFF;
        data[6] = (joint_velocity >> 16) & 0xFF;
        data[7] = (joint_velocity >> 24) & 0xFF;
      }
      else{
        // RCLCPP_WARN(rclcpp::get_logger("SMCHardwareInterface"), "Joint command value not found or undefined command state. Sending Motor Status 3 commands for now.");
        // ENCODING CAN MESSAGE
        data[0] = MOTOR_STATUS_2_CMD;
      }

      // Cast data to uint8_t
      for(int j = 0; j < 8; j++){
        data[j] = static_cast<uint8_t>(data[j]);
        can_tx_frame_.data[j] = data[j];
      }
      canBus.send(can_tx_frame_);
    }
  }
   
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SMCHardwareInterface::perform_command_mode_switch(
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
  RCLCPP_INFO(rclcpp::get_logger("SMCHardwareInterface"), ss.str().c_str());

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
        RCLCPP_INFO(rclcpp::get_logger("SMCHardwareInterface"),
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
        RCLCPP_INFO(rclcpp::get_logger("SMCHardwareInterface"),
          "Joint %s: switched to VELOCITY (cmd vel initialized from state: %f)",
          info_.joints[i].name.c_str(), joint_command_velocity_[i]);
      } else if (requested_modes[i] == integration_level_t::POSITION) {
        joint_command_position_[i] = joint_state_position_[i];
        RCLCPP_INFO(rclcpp::get_logger("SMCHardwareInterface"),
          "Joint %s: switched to POSITION (cmd pos initialized from state: %f)",
          info_.joints[i].name.c_str(), joint_command_position_[i]);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace smc_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  smc_ros2_control::SMCHardwareInterface, hardware_interface::SystemInterface)
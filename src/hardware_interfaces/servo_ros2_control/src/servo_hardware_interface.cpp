#include "servo_ros2_control/servo_hardware_interface.hpp"

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

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace servo_ros2_control
{

double SERVOHardwareInterface::calculate_joint_position_from_motor_position(double motor_position, int gear_ratio){
  // Converts from deg to radians with gear ratio
  return (motor_position * (M_PI/180.0))/gear_ratio;
}

double SERVOHardwareInterface::calculate_joint_displacement_from_motor_position(double motor_position, int gear_ratio, double meters_per_deg){
  // Converts from deg to meters with gear ratio
  return (motor_position * meters_per_deg)/gear_ratio;
}

double SERVOHardwareInterface::calculate_joint_angular_velocity_from_motor_velocity(double motor_velocity, int gear_ratio){
  // Converts from dps to radians/s with gear ratio
  return (motor_velocity * (M_PI/180.0))/gear_ratio;
}

double SERVOHardwareInterface::calculate_joint_linear_velocity_from_motor_velocity(double motor_velocity, int gear_ratio, double meters_per_deg){
  // Converts from dps to meters/s with gear ratio
  return (motor_velocity * meters_per_deg)/gear_ratio;
}

int16_t SERVOHardwareInterface::calculate_motor_position_from_desired_joint_position(double joint_position, int gear_ratio){
  // radians -> deg with gear ratio
  return static_cast<int16_t>(std::round((joint_position*(180/M_PI))*gear_ratio));
}

int16_t SERVOHardwareInterface::calculate_motor_position_from_desired_joint_displacement(double joint_position, int gear_ratio, double meters_per_deg){
  // m -> deg with gear ratio
  return static_cast<int16_t>(std::round((joint_position*meters_per_deg*gear_ratio)));
}

int16_t SERVOHardwareInterface::calculate_motor_velocity_from_desired_joint_angular_velocity(double joint_velocity, int gear_ratio){
  // radians/s -> deg/s with gear ratio
  return static_cast<int16_t>(std::round((joint_velocity*(180/M_PI))*gear_ratio));
}

int16_t SERVOHardwareInterface::calculate_motor_velocity_from_desired_joint_linear_velocity(double joint_velocity, int gear_ratio, double meters_per_deg){
  // m/s -> deg/s with gear ratio
  return static_cast<int16_t>(std::round((joint_velocity*meters_per_deg*gear_ratio)));
}

void SERVOHardwareInterface::logger_function(){

  // Prevents breaking the logger
  if (num_joints == 0) return;

  // Building Message
  std::string log_msg = "\033[2J\033[H \nSERVO Logger";
  std::ostringstream oss;
  std::string status;
  
  // HWI Specific
  oss << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface
      << " | Command CAN ID: 0x" << std::hex << std::uppercase << can_command_id
      << " | Response CAN ID: 0x" << std::hex << std::uppercase << can_response_id
      << " | HWI Update Rate: " << update_rate
      << " | Logger Update Rate: " << logger_rate << "\n"
      << "Elapsed Time since first update: " << elapsed_time << "\n"
      << "\n--- Joint Specific ---";

  for (int i = 0; i < num_joints; i++) {
    switch (device_status[i]) {
      case 0x00:
        status = "IDLE (ready)";
        break;
      case 0x01:
        status = "ACTIVE (busy)";
        break;
      case 0x02:
        status = "DOES NOT EXIST";
        break;
      case 0x03:
        status = "ERROR";
        break;
      default:
        status = "UNDEFINED";
        break;
    }

    oss << "\nJOINT: " << info_.joints[i].name << "\n"
        << "Parameters: Node ID: 0x" << std::hex << std::uppercase << joint_node_ids[i]
        << " | Gear Ratio: " << joint_gear_ratios[i]
        << " | Device Status: " << std::hex << std::uppercase << device_status[i] 
        << " - " << status << "\n"
        << "-- Commands --\n"
        << "Control Mode: " << static_cast<int>(control_level_[i]) << "\n"
        << "Motor Position: " << motor_position[i]
        << " | Joint Command Position: " << joint_command_position_[i] << "\n"
        << "Motor Velocity: " << motor_velocity[i]
        << " | Joint Command Velocity: " << joint_command_velocity_[i] << "\n"
        << "-- State --\n"
        << "Joint Position: " << joint_state_position_[i]
        << " | Joint Velocity: " << joint_state_velocity_[i] << "\n";
  }

  log_msg += oss.str();
  RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"), log_msg.c_str());
}


hardware_interface::CallbackReturn SERVOHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info) // Info stores all parameters in xacro file
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // -- Parameters --
  for (auto& joint : info_.joints) {
    joint_node_ids.push_back(std::clamp(std::stoi(joint.parameters.at("node_id"), nullptr, 0), 0x0, 0xF));
    joint_gear_ratios.push_back(std::abs(std::stoi(joint.parameters.at("gear_ratio"))));

    std::string servo_type = joint.parameters.at("servo_type");
    std::string joint_type = joint.parameters.at("joint_type");
    
    // Standard or Continuous
    if (servo_type == "standard" && joint.parameters.count("max_pos")) {
      servo_type_.push_back(servo_type_t::STANDARD);
    } else if (servo_type == "continuous" && joint.parameters.count("max_vel")) {
      servo_type_.push_back(servo_type_t::CONTINUOUS);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("SERVOHardwareInterface"), "Invalid servo_type parameter for joint %s. Must be 'standard' with 'max_pos' or 'continuous' with 'max_vel'.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Revolute or Prismatic
    if (joint_type == "revolute") {
      joint_type_.push_back(joint_type_t::REVOLUTE);
      meters_per_deg.push_back(std::nan("")); // Not used for revolute joints
    } else if (joint_type == "prismatic" && joint.parameters.count("meters_per_deg")) {
      joint_type_.push_back(joint_type_t::PRISMATIC);
      meters_per_deg.push_back(std::abs(std::stod(joint.parameters.at("meters_per_deg"))));
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("SERVOHardwareInterface"), "Invalid joint_type parameter for joint %s. Must be 'revolute' or 'prismatic'. If prismatic, it must have a 'meters_per_deg' parameter.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Determining rated max based on joint and servo type
    if (joint_type_.back() == joint_type_t::REVOLUTE && servo_type_.back() == servo_type_t::STANDARD) {
      rated_max.push_back(std::abs(std::stod(joint.parameters.at("max_pos")))*(M_PI/180.0)); // Convert from degrees to radians
    } else if(joint_type_.back() == joint_type_t::REVOLUTE && servo_type_.back() == servo_type_t::CONTINUOUS){
      rated_max.push_back(std::abs(std::stod(joint.parameters.at("max_vel")))*(M_PI/180.0)); // Convert from dps to radians/s
    } else if(joint_type_.back() == joint_type_t::PRISMATIC && servo_type_.back() == servo_type_t::STANDARD){
      rated_max.push_back(std::abs(std::stod(joint.parameters.at("max_pos")) * std::stod(joint.parameters.at("meters_per_deg")))); // Convert from degrees to meters
    } else if(joint_type_.back() == joint_type_t::PRISMATIC && servo_type_.back() == servo_type_t::CONTINUOUS){
      rated_max.push_back(std::abs(std::stod(joint.parameters.at("max_vel")) * std::stod(joint.parameters.at("meters_per_deg")))); // Convert from dps to meters/s
    }
    else {
      RCLCPP_ERROR(rclcpp::get_logger("SERVOHardwareInterface"), "Invalid combination of joint_type and servo_type for joint %s.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

  }

  num_joints = static_cast<int>(info_.joints.size());
  update_rate = std::stoi(info_.hardware_parameters.at("update_rate"));
  logger_rate = std::stoi(info_.hardware_parameters.at("logger_rate"));
  logger_state = std::stoi(info_.hardware_parameters.at("logger_state"));
  can_interface = info_.hardware_parameters.at("can_interface");
  can_command_id = std::stoi(info_.hardware_parameters.at("can_id"), nullptr, 0);
  can_response_id = can_command_id+0x01;

  elapsed_update_time = 0.0;
  elapsed_time = 0.0;
  elapsed_logger_time = 0.0;
  
  // -- Command and State Interface initialization --
  joint_state_position_.assign(num_joints, std::numeric_limits<double>::quiet_NaN());
  joint_state_velocity_.assign(num_joints, 0);

  joint_command_position_.assign(num_joints, std::numeric_limits<double>::quiet_NaN());
  joint_command_velocity_.assign(num_joints, 0);

  motor_position.assign(num_joints, 0.0);
  motor_velocity.assign(num_joints, 0.0);
  device_status.assign(num_joints, 0);

  // Set initial control type based on servo type
  control_level_.resize(num_joints, integration_level_t::POSITION);
  for (int i = 0; i < num_joints; i++) {
    if(servo_type_[i] == servo_type_t::STANDARD){
      control_level_[i] = integration_level_t::POSITION;
    } else if(servo_type_[i] == servo_type_t::CONTINUOUS){
      control_level_[i] = integration_level_t::VELOCITY;
    } else{
      control_level_[i] = integration_level_t::UNDEFINED;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn SERVOHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  // Reuse cleanup logic which shuts down the motor and then deinitializes shared pointers.
  // Need this in case on_cleanup never gets called
  return this->on_cleanup(previous_state);
}


std::vector<hardware_interface::StateInterface> SERVOHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Each SERVO motor corresponds to a different joint.
  for(int i = 0; i < num_joints; i++){   
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_state_position_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_state_velocity_[i]));
  }
  
  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface>
SERVOHardwareInterface::export_command_interfaces()
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

hardware_interface::CallbackReturn SERVOHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!canBus.open(can_interface, std::bind(&SERVOHardwareInterface::on_can_message, this, std::placeholders::_1))) {
    RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"), "Failed to open CAN interface");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

void SERVOHardwareInterface::on_can_message(const CANLib::CanFrame& frame) {
  can_rx_frame_ = frame; // Store the received frame for processing in read()

  std::string result;

  int data[7] = {0x00};
  int8_t command_nibble = ((can_rx_frame_.data[0]>>4) & 0x0F);
  int8_t device_id_nibble = (can_rx_frame_.data[0] & 0x0F);
  double raw_motor_position = 0.0;
  double raw_motor_velocity = 0.0;

  for(int i = 0; i < num_joints; i++){
    if(can_rx_frame_.id == can_response_id && command_nibble == MOTOR_STATE_CMD && device_id_nibble == joint_node_ids[i]){
      
      // DECODING CAN MESSAGE FOR VELOCITY
      data[1] = can_rx_frame_.data[1]; // Position low byte
      data[2] = can_rx_frame_.data[2]; // Position high byte
      data[3] = can_rx_frame_.data[3]; // Velocity low byte
      data[4] = can_rx_frame_.data[4]; // Velocity high byte

      // POSITION
      // uint32 -> int16 -> double (for calcs)
      raw_motor_position = static_cast<double>(static_cast<int32_t>((data[2] << 8) | data[1]));

      // VELOCITY
      // uint16 -> int16 -> double (for calcs)
      raw_motor_velocity = static_cast<double>(static_cast<int16_t>((data[4] << 8) | data[3]));

      // CALCULATING JOINT STATE
      if(joint_type_[i] == joint_type_t::REVOLUTE){
        motor_position[i] = calculate_joint_position_from_motor_position(raw_motor_position, joint_gear_ratios[i]); // radians
        motor_velocity[i] = calculate_joint_angular_velocity_from_motor_velocity(raw_motor_velocity, joint_gear_ratios[i]); // radians/s
      }
      else if(joint_type_[i] == joint_type_t::PRISMATIC){
        motor_position[i] = calculate_joint_displacement_from_motor_position(raw_motor_position, joint_gear_ratios[i], meters_per_deg[i]); // meters
        motor_velocity[i] = calculate_joint_linear_velocity_from_motor_velocity(raw_motor_velocity, joint_gear_ratios[i], meters_per_deg[i]); // meters/s
      }
      else{
        RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"), "The joint type for joint %s is undefined.", info_.joints[i].name.c_str());
      }
    }
    else if(can_rx_frame_.id == can_response_id && command_nibble == MOTOR_STATUS_CMD && device_id_nibble == joint_node_ids[i]){
      
      // Populate device status
      device_status[i] = can_rx_frame_.data[1];
    }
    else{
      if(logger_state == 1) {
        // RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"), "Reply not heard.");
      }
    }
  }

}

hardware_interface::CallbackReturn SERVOHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"), "Cleaning up ...please wait...");
  
  // If cleanup occurs before shutdown, this is the last opportunity to shutdown motor since pointers must be deleted here
  int8_t command_nibble = MAINTENANCE_CMD;
  int8_t maintenance_command = 2;  // Motor Shutdown Command
  int8_t device_id_nibble;
  for(int i = 0; i < num_joints; i++){
    can_tx_frame_ = CANLib::CanFrame();
    can_tx_frame_.id = can_command_id;
    can_tx_frame_.dlc = 2;
        
    device_id_nibble = joint_node_ids[i] & 0x0F;
    can_tx_frame_.data[0] = static_cast<uint8_t>((command_nibble << 4) | device_id_nibble);
    can_tx_frame_.data[1] = maintenance_command;
    canBus.send(can_tx_frame_);
  }

  // Close CAN bus
  canBus.close();
  RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"), "Cleaning up successful!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SERVOHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"), "Activating ...please wait...");

  // Sets initial command to joint state
  joint_command_position_ = joint_state_position_;
  
  for (size_t i = 0; i < joint_command_position_.size(); ++i) {
    RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"), "Joint %zu command position initialized to: %f", i, joint_command_position_[i]);
  }

  RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn SERVOHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"), "Deactivating ...please wait...");
  int8_t command_nibble = MAINTENANCE_CMD;
  int8_t maintenance_command = 1;  // Motor Stop Command
  int8_t device_id_nibble;

  for(int i = 0; i < num_joints; i++){
    can_tx_frame_ = CANLib::CanFrame();
    can_tx_frame_.id = can_command_id;
    can_tx_frame_.dlc = 2;
        
    // Motor Stop Command
    device_id_nibble = joint_node_ids[i] & 0x0F;
    can_tx_frame_.data[0] = static_cast<uint8_t>((command_nibble << 4) | device_id_nibble);
    can_tx_frame_.data[1] = maintenance_command;
    canBus.send(can_tx_frame_);
  }

  RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"), "Successfully deactivated all SERVO motors!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type SERVOHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for(int i = 0; i < num_joints; i++) {
    // CALCULATING JOINT STATE
    joint_state_velocity_[i] = motor_velocity[i];
    joint_state_position_[i] = motor_position[i];

    // ACCOUNTING FOR DEVICE STATUS
    if(device_status[i] != 0x00 && device_status[i] != 0x01 && device_status[i] != -1){
      return hardware_interface::return_type::ERROR;
    }
  }
    
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type servo_ros2_control::SERVOHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  elapsed_update_time+=period.seconds();
  double update_period = 1.0/update_rate;
  int data[3] = {0x00};
  int8_t command_nibble;
  int8_t device_id_nibble;
  int16_t joint_angle = 0;
  int16_t joint_velocity = 0;

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
    for(int i = 0; i < num_joints; i++) {
      // Motor Command for each joint at given frequency
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = can_command_id;
      can_tx_frame_.dlc = 3;
      
      if(control_level_[i] == integration_level_t::POSITION && servo_type_[i] == servo_type_t::STANDARD && std::isfinite(joint_command_position_[i])) {
        // CALCULATE DESIRED JOINT ANGLE
        joint_angle = std::clamp(joint_command_position_[i], 0.0, rated_max[i]); // Input must be within bounds of rated max (positive and either rad or m)
        if(joint_type_[i] == joint_type_t::REVOLUTE){
          joint_angle = calculate_motor_position_from_desired_joint_position(joint_angle, joint_gear_ratios[i]);
        }
        else if(joint_type_[i] == joint_type_t::PRISMATIC){
          joint_angle = calculate_motor_position_from_desired_joint_displacement(joint_angle, joint_gear_ratios[i], meters_per_deg[i]);
        }
        else{
          RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"), "The joint type for joint %s is undefined.", info_.joints[i].name.c_str());
        }
       
        // ENCODING CAN MESSAGE
        data[0] = ABSOLUTE_POS_CONTROL_CMD + joint_node_ids[i];
        data[1] = joint_angle & 0xFF;
        data[2] = (joint_angle >> 8) & 0xFF;

        // Cast data to uint8_t
        for(int j = 0; j < 3; j++){
          data[j] = static_cast<uint8_t>(data[j]);
          can_tx_frame_.data[j] = data[j];
        }
        
      }
      else if(control_level_[i] == integration_level_t::VELOCITY && servo_type_[i] == servo_type_t::CONTINUOUS && std::isfinite(joint_command_velocity_[i])) {
        
        // CALCULATE DESIRED JOINT VELOCITY
        joint_velocity = std::clamp(joint_command_velocity_[i], -rated_max[i], rated_max[i]); // Input must be within bounds of rated max (all real values and either rad or m)
        if(joint_type_[i] == joint_type_t::REVOLUTE){
          joint_velocity = calculate_motor_velocity_from_desired_joint_angular_velocity(joint_velocity, joint_gear_ratios[i]);
        }
        else if(joint_type_[i] == joint_type_t::PRISMATIC){
          joint_velocity = calculate_motor_velocity_from_desired_joint_linear_velocity(joint_velocity, joint_gear_ratios[i], meters_per_deg[i]);
        }
        else{
          RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"), "The joint type for joint %s is undefined.", info_.joints[i].name.c_str());
        }
       
        // ENCODING CAN MESSAGE
        data[0] = VELOCITY_CONTROL_CMD + joint_node_ids[i];
        data[1] = joint_velocity & 0xFF;
        data[2] = (joint_velocity >> 8) & 0xFF;

        // Cast data to uint8_t
        for(int j = 0; j < 3; j++){
          data[j] = static_cast<uint8_t>(data[j]);
          can_tx_frame_.data[j] = data[j];
        }

      }
      else {
        command_nibble = MOTOR_STATE_CMD;       // Motor State Command for each joint at given frequency
        can_tx_frame_.dlc = 1;
        device_id_nibble = joint_node_ids[i] & 0x0F;
        can_tx_frame_.data = { static_cast<uint8_t>((command_nibble << 4) | device_id_nibble) };
      
        if (control_level_[i] == integration_level_t::POSITION && servo_type_[i] == servo_type_t::CONTINUOUS) {
          if (logger_state == 1) {
            RCLCPP_WARN(rclcpp::get_logger("SERVOHardwareInterface"), "Joint %s is continuous type and cannot be position controlled.", info_.joints[i].name.c_str());
          }
        }
        else if (control_level_[i] == integration_level_t::VELOCITY && servo_type_[i] == servo_type_t::STANDARD) {
          if (logger_state == 1) {
            RCLCPP_WARN(rclcpp::get_logger("SERVOHardwareInterface"), "Joint %s is standard type and cannot be velocity controlled.", info_.joints[i].name.c_str());
          }
        }
        else{
          if (logger_state == 1) {
            RCLCPP_WARN(rclcpp::get_logger("SERVOHardwareInterface"), "Joint command value not found or undefined command state");
          }
        }
      }
      
      canBus.send(can_tx_frame_);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SERVOHardwareInterface::perform_command_mode_switch(
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
  RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"), ss.str().c_str());

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
        RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"),
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
        RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"),
          "Joint %s: switched to VELOCITY (cmd vel initialized from state: %f)",
          info_.joints[i].name.c_str(), joint_command_velocity_[i]);
      } else if (requested_modes[i] == integration_level_t::POSITION) {
        joint_command_position_[i] = joint_state_position_[i];
        RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"),
          "Joint %s: switched to POSITION (cmd pos initialized from state: %f)",
          info_.joints[i].name.c_str(), joint_command_position_[i]);
      }
    }
  }

  return hardware_interface::return_type::OK;
}


}  // namespace servo_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  servo_ros2_control::SERVOHardwareInterface, hardware_interface::SystemInterface)
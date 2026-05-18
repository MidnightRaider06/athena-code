#include "rmd_ros2_control/rmd_hardware_interface.hpp"

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
#include <bit>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rmd_ros2_control
{

// -- HELPER FUNCTIONS -- //

double RMDHardwareInterface::calculate_joint_position_from_motor_position(double motor_position, int gear_ratio){
  // Converts from 0.01 deg to deg to radians/s
  return (motor_position * 0.01 * (M_PI/180.0))/gear_ratio;
}

double RMDHardwareInterface::calculate_joint_velocity_from_motor_velocity(double motor_velocity, int gear_ratio){
  // Converts from dps to radians/s
  return (motor_velocity * (M_PI/180.0))/gear_ratio;
}

int32_t RMDHardwareInterface::calculate_motor_position_from_desired_joint_position(double joint_position, int gear_ratio){
  // radians -> deg -> 0.01 deg
  return static_cast<int32_t>(std::round((joint_position*(180/M_PI)*100)*gear_ratio));
}

int32_t RMDHardwareInterface::calculate_motor_velocity_from_desired_joint_velocity(double joint_velocity, int gear_ratio){
  // radians/s -> deg/s -> 0.01 deg/s
  return static_cast<int32_t>(std::round((joint_velocity*(180/M_PI)*100)*gear_ratio));
}

void RMDHardwareInterface::send_command(int can_id, int cmd_id){
  CANLib::CanFrame frame;
  frame.id  = can_id;
  frame.dlc = 8;
  frame.data.fill(0);
  frame.data[0] = cmd_id;
  canBus.send(frame);
}

void RMDHardwareInterface::logger_function(){

  // Prevents breaking the logger
  if (RMDJoints_.size() == 0) return;

  // Building Message
  std::string log_msg = "\033[2J\033[H \nRMD Logger";
  std::string control_mode = "";
  
  std::ostringstream oss;
  oss << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface
      << " | HWI Update Rate: " << update_rate
      << " | Logger Update Rate: " << logger_rate << "\n"
      << "Elapsed Time since first update: " << elapsed_time << "\n"
      << "\n--- Joint Specific ---";

  for (auto & joint : RMDJoints_) {
    if(static_cast<int>(joint.control_level) == 1) {
      control_mode = "POSITION";
    }
    else if(static_cast<int>(joint.control_level) == 2) {
      control_mode = "VELOCITY";
    }
    else {
      control_mode = "UNDEFINED";
    }
    oss << "\n----- JOINT: " << joint.name << " -----\n"
        << "Parameters: Write CAN ID: 0x" << std::hex << std::uppercase << joint.node_write_id
        << " | Read CAN ID: 0x" << std::hex << std::uppercase << joint.node_read_id
        << " | Gear Ratio: " << joint.gear_ratio
        << " | Orientation: " << joint.orientation << "\n"
        << "Curr P: " << joint.current_Kp 
        << " | Curr I: " << joint.current_Ki
        << " | Speed P: " << joint.speed_Kp
        << " | Speed I: " << joint.speed_Ki
        << " | Pos P: " << joint.position_Kp
        << " | Pos I: " << joint.position_Ki
        << "\n-- Commands --\n"
        << "Control Mode: " << control_mode << "\n"
        << "Motor Position: " << joint.motor_position
        << " | Joint Command Position: " << joint.joint_command_position << "\n"
        << "Motor Velocity: " << joint.motor_velocity
        << " | Joint Command Velocity: " << joint.joint_command_velocity << "\n"
        << "Motor Status Request: " << joint.motor_status_req
        << " | Motor Maintenance Request Rate: " << joint.motor_maintenance_req << "\n"
        << " | Motor Maintenance Command High: " << joint.maintenance_frame_high
        << " | Motor Maintenance Command Low: " << joint.maintenance_frame_low
        << " | Motor Maintenance Command Full: " << joint.maintenance_frame << "\n"
        << "Previous: Motor Status Request Rate: " << joint.prev_status_req
        << " | Motor Maintenance Request Rate: " << joint.prev_maintenance_req
        << " Decoded Maintenance Frame: ";
        for (const auto& byte : joint.decoded_maintenance_frame) {
          oss << std::hex << std::uppercase << static_cast<int>(byte) << " ";
        }
    oss << "\n-- State --\n"
        << "Joint Position: " << joint.joint_state_position << " rad"
        << " | Joint Velocity: " << joint.joint_state_velocity << " rad/s"
        << " | Joint Acceleration: " << joint.acceleration << " dps/s\n"
        << "-- Telemetry --\n"
        << "Motor Temperature: " << joint.motor_temperature << " C"
        << " | Torque Current: " << joint.motor_torque_current << " A"
        << " | Motor Status: : " << joint.motor_status << "\n";

    log_msg += oss.str();
    RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), log_msg.c_str());
  }
}

bool RMDHardwareInterface::process_status(uint16_t status, const rclcpp::Logger & logger)
{
  bool has_warning = false;
  bool has_fatal_error = false;

  std::ostringstream error_stream;

  // Most safe state, no errors or warnings
  if (status == 0) {
    return true;
  }

  for (const auto& entry : status_table) {
    uint16_t code = static_cast<uint16_t>(entry.flag);

    if (code == 0) {
      continue;
    }

    if (status & code) {

      // warning case
      if (code == 0x0001) {
        has_warning = true;
        continue;
      }

      // fatal error case
      if (code > 1) {
        has_fatal_error = true;
        error_stream << entry.name << " (0x"
                     << std::hex << code << std::dec << "), ";
      }
    }
  }

  // WARN
  if (has_warning) {
    // RCLCPP_WARN(logger, "Brake Released (warning state)");
  }

  // ERROR
  if (has_fatal_error) {
    RCLCPP_ERROR(logger, "Motor Errors: %s", error_stream.str().c_str());
  }

  // return system safety state
  return !has_fatal_error;
}

bool RMDHardwareInterface::format_maintenance_command(CANLib::CanFrame & frame, const DecodedCommand & decoded_cmd) {
  std::fill(std::begin(frame.data), std::end(frame.data), 0x00);
  frame.data[0] = decoded_cmd.command_id; // Set multiplexor
  switch (static_cast<MaintenanceCommands>(decoded_cmd.command_id)) {
    case MaintenanceCommands::WRITE_PID_TO_RAM_CMD:
      if(decoded_cmd.u8_data.size() != 6 || decoded_cmd.i16_data.size() != 0 || decoded_cmd.i32_data.size() != 0){
        return false; // Invalid data format for this command
      }
      else{
        frame.data[2] = decoded_cmd.u8_data[0]; // Current Loop P gain
        frame.data[3] = decoded_cmd.u8_data[1]; // Current Loop I gain
        frame.data[4] = decoded_cmd.u8_data[2]; // Speed Loop P gain
        frame.data[5] = decoded_cmd.u8_data[3]; // Speed Loop I gain
        frame.data[6] = decoded_cmd.u8_data[4]; // Position Loop P gain
        frame.data[7] = decoded_cmd.u8_data[5]; // Position Loop I gain
        return true;
      }
    case MaintenanceCommands::WRITE_PID_TO_ROM_CMD:
      if(decoded_cmd.u8_data.size() != 6 || decoded_cmd.i16_data.size() != 0 || decoded_cmd.i32_data.size() != 0){
        return false; // Invalid data format for this command
      }
      else{
        frame.data[2] = decoded_cmd.u8_data[0]; // Current Loop P gain
        frame.data[3] = decoded_cmd.u8_data[1]; // Current Loop I gain
        frame.data[4] = decoded_cmd.u8_data[2]; // Speed Loop P gain
        frame.data[5] = decoded_cmd.u8_data[3]; // Speed Loop I gain
        frame.data[6] = decoded_cmd.u8_data[4]; // Position Loop P gain
        frame.data[7] = decoded_cmd.u8_data[5]; // Position Loop I gain
        return true;
      }
    case MaintenanceCommands::WRITE_ACCELERATION_CMD:
      if(decoded_cmd.u8_data.size() != 1 || decoded_cmd.i16_data.size() != 0 || decoded_cmd.i32_data.size() != 1){
        return false; // Invalid data format for this command
      }
      else{
        frame.data[1] = decoded_cmd.u8_data[0]; // Function Index
        frame.data[4] = decoded_cmd.i32_data[0] & 0xFF; // Acceleration low byte
        frame.data[5] = (decoded_cmd.i32_data[0] >> 8) & 0xFF; // Acceleration byte 2
        frame.data[6] = (decoded_cmd.i32_data[0] >> 16) & 0xFF; // Acceleration byte 3
        frame.data[7] = (decoded_cmd.i32_data[0] >> 24) & 0xFF; // Acceleration high byte
        return true;
      }
    case MaintenanceCommands::WRITE_ENCODER_MULTI_TURN_ZERO_CMD:
      if(decoded_cmd.u8_data.size() != 0 || decoded_cmd.i16_data.size() != 0 || decoded_cmd.i32_data.size() != 1){
        return false; // Invalid data format for this command
      }
      else{
        frame.data[4] = decoded_cmd.i32_data[0] & 0xFF; // Encoder offset low byte
        frame.data[5] = (decoded_cmd.i32_data[0] >> 8) & 0xFF; // Encoder offset byte 2
        frame.data[6] = (decoded_cmd.i32_data[0] >> 16) & 0xFF; // Encoder offset byte 3
        frame.data[7] = (decoded_cmd.i32_data[0] >> 24) & 0xFF; // Encoder offset high byte
        return true;
      }
    case MaintenanceCommands::WRITE_CURRENT_MULTI_TURN_POS_ZERO_CMD:
      if(decoded_cmd.u8_data.size() != 0 || decoded_cmd.i16_data.size() != 0 || decoded_cmd.i32_data.size() != 0){
        return false; // Invalid data format for this command
      }
      else{
        // Empty
        return true;
      }
    case MaintenanceCommands::SYSTEM_RESET_CMD:
      if(decoded_cmd.u8_data.size() != 0 || decoded_cmd.i16_data.size() != 0 || decoded_cmd.i32_data.size() != 0){
        return false; // Invalid data format for this command
      }
      else{
        // Empty
        return true;
      }
    case MaintenanceCommands::BRAKE_RELEASE_CMD:
      if(decoded_cmd.u8_data.size() != 0 || decoded_cmd.i16_data.size() != 0 || decoded_cmd.i32_data.size() != 0){
        return false; // Invalid data format for this command
      }
      else{
        // Empty
        return true;
      }
    case MaintenanceCommands::BRAKE_LOCK_CMD:
      if(decoded_cmd.u8_data.size() != 0 || decoded_cmd.i16_data.size() != 0 || decoded_cmd.i32_data.size() != 0){
        return false; // Invalid data format for this command
      }
      else{
        // Empty
        return true;
      }
    case MaintenanceCommands::MOTOR_SHUTDOWN_CMD:
      if(decoded_cmd.u8_data.size() != 0 || decoded_cmd.i16_data.size() != 0 || decoded_cmd.i32_data.size() != 0){
        return false; // Invalid data format for this command
      }
      else{
        // Empty
        return true;
      }
    case MaintenanceCommands::MOTOR_STOP_CMD:
      if(decoded_cmd.u8_data.size() != 0 || decoded_cmd.i16_data.size() != 0 || decoded_cmd.i32_data.size() != 0){
        return false; // Invalid data format for this command
      }
      else{
        // Empty
        return true;
      }
    default:
      return false;
  }
}

void RMDHardwareInterface::format_control_command(CANLib::CanFrame & frame, RMDJoint & joint) {
  std::fill(std::begin(frame.data), std::end(frame.data), 0x00);
  if (joint.control_level == integration_level_t::POSITION &&
    std::isfinite(joint.joint_command_position) &&
    joint.joint_command_position != joint.prev_joint_command_position)
  {
    int32_t joint_angle = joint.orientation *
      calculate_motor_position_from_desired_joint_position(
        joint.joint_command_position, joint.gear_ratio);

    frame.data[0] = static_cast<uint8_t>(ControlCommands::ABSOLUTE_POS_CONTROL_CMD);
    frame.data[1] = 0x00;
    frame.data[2] = joint.operating_velocity & 0xFF;
    frame.data[3] = (joint.operating_velocity >> 8) & 0xFF;
    frame.data[4] = joint_angle & 0xFF;
    frame.data[5] = (joint_angle >> 8) & 0xFF;
    frame.data[6] = (joint_angle >> 16) & 0xFF;
    frame.data[7] = (joint_angle >> 24) & 0xFF;

    joint.prev_joint_command_position = joint.joint_command_position;
  }
  else if (joint.control_level == integration_level_t::VELOCITY &&
          std::isfinite(joint.joint_command_velocity) &&
          joint.joint_command_velocity != joint.prev_joint_command_velocity)
  {
    int32_t joint_velocity = joint.orientation *
      calculate_motor_velocity_from_desired_joint_velocity(
        joint.joint_command_velocity, joint.gear_ratio);

    frame.data[0] = static_cast<uint8_t>(ControlCommands::SPEED_CONTROL_CMD);
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = joint_velocity & 0xFF;
    frame.data[5] = (joint_velocity >> 8) & 0xFF;
    frame.data[6] = (joint_velocity >> 16) & 0xFF;
    frame.data[7] = (joint_velocity >> 24) & 0xFF;

    joint.prev_joint_command_velocity = joint.joint_command_velocity;
  }
  else
  {
    frame.data[0] = static_cast<uint8_t>(StatusCommands::MOTOR_STATUS_2_CMD);
  }
}

// -- LIFECYCLE CALLBACKS --

hardware_interface::CallbackReturn RMDHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info) // Info stores all parameters in xacro file
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Setting Parameters
  RMDJoints_.clear();

  num_joints = static_cast<int>(RMDJoints_.size());

  // General HWI parameters
  update_rate = std::stoi(info_.hardware_parameters.at("update_rate"));
  logger_rate = std::stoi(info_.hardware_parameters.at("logger_rate"));
  logger_state = std::stoi(info_.hardware_parameters.at("logger_state"));
  can_interface = info_.hardware_parameters.at("can_interface");

  // Per Joint Parameters
  for (auto& joint : info_.joints) {
    // Collect joint specific parameters
    int write_id = std::clamp(std::stoi(joint.parameters.at("node_id"), nullptr, 0), 0x141, 0x160);
    int gear_ratio = std::abs(std::stoi(joint.parameters.at("gear_ratio")));
    int orientation = std::stoi(joint.parameters.at("joint_orientation")) == -1 ? -1 : 1;
    int operating_velocity = std::clamp(std::stoi(joint.parameters.at("operating_velocity")), 0, 65*gear_ratio);

    // Collect state interface names
    std::vector<std::string> state_if_names;
    for (const auto &si : joint.state_interfaces) {
      state_if_names.push_back(si.name);
    }

    // Collect command interface names
    std::vector<std::string> command_if_names;
    for (const auto &ci : joint.command_interfaces) {
      command_if_names.push_back(ci.name);
    }

    // Copy parameters into an unordered_map<string,string>
    std::unordered_map<std::string, std::string> params_map;
    for (const auto &p : joint.parameters) {
      params_map.emplace(p.first, p.second);
    }

    // Populate each RMDJoint object
    RMDJoints_.push_back(
      RMDJoint{
        .name = joint.name,
        .node_write_id = static_cast<uint32_t>(write_id),
        .node_read_id = static_cast<uint32_t>(write_id + 0x100),
        .gear_ratio = gear_ratio,
        .orientation = orientation,
        .operating_velocity = static_cast<uint16_t>(operating_velocity),
        .control_level = integration_level_t::POSITION,
        .joint_state_position = std::numeric_limits<double>::quiet_NaN(),
        .joint_state_velocity = 0.0,
        .motor_temperature = std::numeric_limits<double>::quiet_NaN(),
        .motor_torque_current = std::numeric_limits<double>::quiet_NaN(),
        .motor_status = std::numeric_limits<double>::quiet_NaN(),
        .current_Kp = std::numeric_limits<double>::quiet_NaN(),
        .current_Ki = std::numeric_limits<double>::quiet_NaN(),
        .speed_Kp = std::numeric_limits<double>::quiet_NaN(),
        .speed_Ki = std::numeric_limits<double>::quiet_NaN(),
        .position_Kp = std::numeric_limits<double>::quiet_NaN(),
        .position_Ki = std::numeric_limits<double>::quiet_NaN(),
        .acceleration = std::numeric_limits<double>::quiet_NaN(),

        .joint_command_position = std::numeric_limits<double>::quiet_NaN(),
        .joint_command_velocity = 0.0,
        .motor_status_req = 0.0,
        .motor_maintenance_req = 0.0,
        .maintenance_frame_high = 0.0,
        .maintenance_frame_low = 0.0,
        .maintenance_frame = 0.0,
        .maintenance_data_count = 0.0,
        .decoded_maintenance_frame = {},

        .prev_status_req = 0.0,
        .prev_maintenance_req = 0.0,
        .elapsed_status_request_time = 0.0,
        .elapsed_maintenance_request_time = 0.0,
        .motor_velocity = 0.0,
        .motor_position = 0.0,
        .prev_joint_command_position = std::numeric_limits<double>::quiet_NaN(),
        .prev_joint_command_velocity = std::numeric_limits<double>::quiet_NaN(),
        .state_interface_names = state_if_names,
        .command_interface_names = command_if_names,
        .parameters = params_map
      }
    );
  }

  // Initialize timing tools
  elapsed_update_time = 0.0;
  elapsed_time = 0.0;
  elapsed_logger_time = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn RMDHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  // Reuse cleanup logic which shuts down the motor and then deinitializes shared pointers.
  // Need this in case on_cleanup never gets called
  return this->on_cleanup(previous_state);
}


std::vector<hardware_interface::StateInterface> RMDHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Each RMD motor corresponds to a different joint.
  for (auto & joint : RMDJoints_) {
    for (auto & iface : joint.state_interface_names) {
      double * val = nullptr;
      if (iface == "position") {
        val = &joint.joint_state_position;
      } else if (iface == "velocity") {
        val = &joint.joint_state_velocity;
      } else if (iface == "motor_temperature") {
        val = &joint.motor_temperature;
      } else if (iface == "torque_current") {
        val = &joint.motor_torque_current;
      } else if (iface == "status") {
        val = &joint.motor_status;
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("RMDHardwareInterface"),
          "Unknown state interface '%s' for joint '%s'",
          iface.c_str(), joint.name.c_str());
        continue;
      }
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, iface, val));
    }
  }
  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface>
RMDHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & joint : RMDJoints_) {
    for (auto & iface : joint.command_interface_names) {
      double * val = nullptr;
      if (iface == "position") {
        val = &joint.joint_command_position;
      } else if (iface == "velocity") {
        val = &joint.joint_command_velocity;
      } else if (iface == "status_request") {
        val = &joint.motor_status_req;
      } else if (iface == "maintenance_request") {
        val = &joint.motor_maintenance_req;
      } else if (iface == "maintenance_frame_high") {
        val = &joint.maintenance_frame_high;
      } else if (iface == "maintenance_frame_low") {
        val = &joint.maintenance_frame_low;
      } else if (iface == "maintenance_data_count") {
        val = &joint.maintenance_data_count;
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("RMDHardwareInterface"), 
          "Unknown command interface '%s' for joint '%s'", 
          iface.c_str(), joint.name.c_str());
        continue;
      }
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, iface, val));
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RMDHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!canBus.open(can_interface, std::bind(&RMDHardwareInterface::on_can_message, this, std::placeholders::_1))) {
    RCLCPP_ERROR(rclcpp::get_logger("RMDHardwareInterface"), "Failed to open CAN interface");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

void RMDHardwareInterface::on_can_message(const CANLib::CanFrame& frame) {
  can_rx_frame_ = frame; // Store the received frame for processing in read()

  std::string result;

  std::vector<int> data(8, 0x00);

  // Decode CAN Message
  for (auto & joint : RMDJoints_) {
    if(can_rx_frame_.id == joint.node_read_id && 
      (can_rx_frame_.data[0] == static_cast<uint8_t>(ControlCommands::SPEED_CONTROL_CMD)  || 
       can_rx_frame_.data[0] == static_cast<uint8_t>(ControlCommands::ABSOLUTE_POS_CONTROL_CMD)  || 
       can_rx_frame_.data[0] == static_cast<uint8_t>(StatusCommands::MOTOR_STATUS_2_CMD))) {
    
      // Speed Control, Absolute Position Control, and Motor Status 2 all share the same reply  
      data[0] = can_rx_frame_.data[0];
      data[1] = can_rx_frame_.data[1]; // Motor Temperature
      data[2] = can_rx_frame_.data[2]; // Torque low byte
      data[3] = can_rx_frame_.data[3]; // Torque high byte
      data[4] = can_rx_frame_.data[4]; // speed low byte
      data[5] = can_rx_frame_.data[5]; // speed high byte
      data[6] = can_rx_frame_.data[6]; // encoder position low byte
      data[7] = can_rx_frame_.data[7]; // encoder position high byte

      // TEMPERATURE (1 byte, degrees C)
      joint.motor_temperature = static_cast<double>(data[1]);

      // TORQUE CURRENT (16-bit signed, 0.01A per LSB)
      joint.motor_torque_current = static_cast<double>(static_cast<int16_t>((data[3] << 8) | data[2])) * 0.01;

      // POSITION 
      // uint16 -> int16 -> double (for calcs)
      joint.motor_position = static_cast<double>(static_cast<int16_t>((data[7] << 8) | data[6]));

      // VELOCITY
      // uint16 -> int16 -> double (for calcs)
      joint.motor_velocity = static_cast<double>(static_cast<int16_t>((data[5] << 8) | data[4]));
    } else if(can_rx_frame_.id == joint.node_read_id && 
              can_rx_frame_.data[0] == static_cast<uint8_t>(StatusCommands::MOTOR_STATUS_1_CMD)) {

      // DECODING Motor Status 1 Message
      data[0] = can_rx_frame_.data[0];
      data[1] = can_rx_frame_.data[1]; // Motor Temperature
      data[2] = 0x00;                  // Null
      data[3] = can_rx_frame_.data[3]; // Brake State
      data[4] = can_rx_frame_.data[4]; // Voltage low byte
      data[5] = can_rx_frame_.data[5]; // Voltage high byte
      data[6] = can_rx_frame_.data[6]; // Error state low byte
      data[7] = can_rx_frame_.data[7]; // Error state high byte

      // TEMPERATURE (degrees C)
      joint.motor_temperature = static_cast<double>(data[1]);
      
      // Initialize motor status command
      joint.motor_status = 0.0;

      // BRAKE STATUS (0: Locked, 1: Released)
      joint.motor_status = joint.motor_status + static_cast<double>(data[3]);
      
      // ERROR STATE
      joint.motor_status = joint.motor_status + static_cast<double>(static_cast<int16_t>((data[7] << 8) | data[6]));
    } else if(can_rx_frame_.id == joint.node_read_id && 
              can_rx_frame_.data[0] == static_cast<uint8_t>(StatusCommands::READ_PID_CMD)) {
      RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Received PID Gains from joint: %s", joint.name.c_str());
      data[1] = 0x00; // Null
      data[2] = can_rx_frame_.data[2]; // Current Loop P gain
      data[3] = can_rx_frame_.data[3]; // Current Loop I gain
      data[4] = can_rx_frame_.data[4]; // Speed Loop P gain
      data[5] = can_rx_frame_.data[5]; // Speed Loop I gain
      data[6] = can_rx_frame_.data[6]; // Position Loop P gain
      data[7] = can_rx_frame_.data[7]; // Position Loop I gain
      
      // Joint PI Gains
      joint.current_Kp = static_cast<double>(data[2]);
      joint.current_Ki = static_cast<double>(data[3]);
      joint.speed_Kp = static_cast<double>(data[4]);
      joint.speed_Ki = static_cast<double>(data[5]);
      joint.position_Kp = static_cast<double>(data[6]);
      joint.position_Ki = static_cast<double>(data[7]);
    }
    else if(can_rx_frame_.id == joint.node_read_id && 
            can_rx_frame_.data[0] == static_cast<uint8_t>(StatusCommands::READ_ACCELERATION_CMD)) {
      RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Received Acceleration from joint: %s", joint.name.c_str());
      data[1] = 0x00; // Null
      data[2] = 0x00; // Null
      data[3] = 0x00; // Null
      data[4] = can_rx_frame_.data[4]; // Acceleration low byte
      data[5] = can_rx_frame_.data[5]; // Acceleration byte 2
      data[6] = can_rx_frame_.data[6]; // Acceleration byte 3
      data[7] = can_rx_frame_.data[7]; // Acceleration high byte
      
      // Joint acceleration in dps/s
      joint.acceleration = static_cast<double>((data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4]);
    }
    else {
      if(logger_state == 1) {
        // RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Reply not heard.");
      }
    }
  }
}

hardware_interface::CallbackReturn RMDHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // If cleanup occurs before shutdown, this is the last opportunity to shutdown motor since pointers must be deleted here
    for (const auto & joint : RMDJoints_) {
    send_command(joint.node_write_id, static_cast<int>(MaintenanceCommands::MOTOR_SHUTDOWN_CMD)); // Motor Shutdown Command
    send_command(joint.node_write_id, static_cast<int>(MaintenanceCommands::BRAKE_LOCK_CMD)); // Brake Lock Command (don't think this works)
  }

  canBus.close();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RMDHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Activating ...please wait...");

  for (const auto & joint : RMDJoints_) {
    // Brake Release command (pretty sure brakes don't work)
    send_command(joint.node_write_id, static_cast<int>(MaintenanceCommands::BRAKE_RELEASE_CMD));
  }

  // Sets initial command to joint state
  for (auto & joint : RMDJoints_) {
    joint.joint_command_position = joint.joint_state_position;
  }

  RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn RMDHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Deactivating ...please wait...");

  for (const auto & joint : RMDJoints_) {
    send_command(joint.node_write_id, static_cast<int>(MaintenanceCommands::MOTOR_STOP_CMD)); // Motor Stop Command
    send_command(joint.node_write_id, static_cast<int>(MaintenanceCommands::BRAKE_LOCK_CMD)); // Brake Lock Command (don't think this works)
  }

  RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Successfully deactivated all RMD motors!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type RMDHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{  
  // CALCULATING JOINT STATE
  for (auto & joint : RMDJoints_) {
    joint.joint_state_velocity = calculate_joint_velocity_from_motor_velocity(joint.motor_velocity, joint.gear_ratio);
    joint.joint_state_position = calculate_joint_position_from_motor_position(joint.motor_position, joint.gear_ratio);

    // Process status logs warning and errors and returns false if fatal error occurs
    if (!process_status(joint.motor_status, rclcpp::get_logger("RMDHardwareInterface"))) {
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type rmd_ros2_control::RMDHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Logger update
  elapsed_time+=period.seconds();
  elapsed_logger_time+=period.seconds();
  double logging_period = 1.0/logger_rate;
  if(elapsed_logger_time > logging_period){
    elapsed_logger_time = 0.0;
    if (logger_state == 1) {
      logger_function();
    }
  }

  // Status request handling
  for (auto & joint : RMDJoints_) {
    double curr_status_req = joint.motor_status_req;
    if (curr_status_req < 0 && joint.prev_status_req >= 0) // Send one shot status request
    {
      for (auto & status_cmd : kStatusCommands) {
        send_command(joint.node_write_id, static_cast<int>(status_cmd));
      }
      RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "One-shot status request sent for joint '%s'.", joint.name.c_str());
    }
    else if (curr_status_req > 0) // Send status request at specified frequency (Hz)
    {
      joint.elapsed_status_request_time += period.seconds();
      double status_request_period = 1.0 / curr_status_req;
      if(joint.elapsed_status_request_time > status_request_period){
        joint.elapsed_status_request_time = 0.0;
        for (auto & status_cmd : kStatusCommands) {
          send_command(joint.node_write_id, static_cast<int>(status_cmd));
        }
      }
    }
    joint.prev_status_req = curr_status_req;
  }

  // Maintenance request handling
  for (auto & joint : RMDJoints_) {
    can_tx_frame_ = CANLib::CanFrame();
    can_tx_frame_.id = joint.node_write_id;
    can_tx_frame_.dlc = 8;

    auto doubles_to_payload = [](double high, double low) -> int64_t
    {
      uint64_t h = static_cast<uint64_t>(high);
      uint64_t l = static_cast<uint64_t>(low);
      return static_cast<int64_t>((h << 32) | l);
    };

    // Call it and store the result
    joint.maintenance_frame = doubles_to_payload(joint.maintenance_frame_high, joint.maintenance_frame_low);

    // Decode the maintenance command interfaces
    DecodedCommand decoded_maintenance_cmd = unpack_command_full(static_cast<uint32_t>(joint.maintenance_data_count), static_cast<uint64_t>(joint.maintenance_frame));

    pack_decoded_maintenance_frame(joint, decoded_maintenance_cmd); // Store decoded maintenance command in joint struct for logging and validation

    if(!format_maintenance_command(can_tx_frame_, decoded_maintenance_cmd)){ // Validate maintenance command ID before sending
      // RCLCPP_WARN(rclcpp::get_logger("RMDHardwareInterface"), "Invalid maintenance command for joint '%s'.", joint.name.c_str());
      continue;
    }

    double curr_maintenance_req = joint.motor_maintenance_req;

    if (curr_maintenance_req < 0 && joint.prev_maintenance_req >= 0) // Send one shot maintenance request
    {
      canBus.send(can_tx_frame_);
      RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "One-shot maintenance request sent for joint '%s'.", joint.name.c_str());
    }
    else if(curr_maintenance_req > 0) // Send maintenance request at specified frequency (Hz)
    {
      joint.elapsed_maintenance_request_time += period.seconds();
      double maintenance_request_period = 1.0 / joint.motor_maintenance_req;
      if(joint.elapsed_maintenance_request_time > maintenance_request_period){
        joint.elapsed_maintenance_request_time = 0.0;
        canBus.send(can_tx_frame_);
      }
    }
    joint.prev_maintenance_req = curr_maintenance_req;
  }

  // HWI can only go as fast as the controller manager. To limit frequency of bus messages,
  // keep track of time passed over iterations of this function and if it exceeds the 
  // desired frequency of the HWI, skip message
  elapsed_update_time+=period.seconds();
  double update_period = 1.0/update_rate;
  if(elapsed_update_time > update_period){
    elapsed_update_time = 0.0;
    for(auto & joint : RMDJoints_) {
      can_tx_frame_ = CANLib::CanFrame(); // Must reinstantiate else data from past iteration gets repeated
      can_tx_frame_.id = joint.node_write_id;
      can_tx_frame_.dlc = 8;
      
      format_control_command(can_tx_frame_, joint);
      canBus.send(can_tx_frame_);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RMDHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string>& start_interfaces,
  const std::vector<std::string>& stop_interfaces)
{
  // Debug: print incoming requests
  // std::ostringstream ss;
  // ss << "perform_command_mode_switch called. start_interfaces: [";
  // for (auto &s : start_interfaces) ss << s << ",";
  // ss << "] stop_interfaces: [";
  // for (auto &s : stop_interfaces) ss << s << ",";
  // ss << "]";
  // RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), ss.str().c_str());

  // For each joint, decide its new control mode based on start/stop interfaces.
  // We allow partial starts/stops: only affected joints are switched.
  std::vector<integration_level_t> requested_modes(RMDJoints_.size(), integration_level_t::UNDEFINED);

  // Process stop interfaces first: mark those joints as UNDEFINED
  for (const auto &ifname : stop_interfaces) {
    for (size_t i = 0; i < RMDJoints_.size(); ++i){
      auto & joint = RMDJoints_[i];
      const std::string pos_if = joint.name + "/" + std::string(hardware_interface::HW_IF_POSITION);
      const std::string vel_if = joint.name + "/" + std::string(hardware_interface::HW_IF_VELOCITY);
      if (ifname == pos_if || ifname == vel_if || ifname.find(joint.name) != std::string::npos) {
        requested_modes[i] = integration_level_t::UNDEFINED;
      }
    }
  }

  // Process start interfaces: set POSITION or VELOCITY per joint
  for (const auto &ifname : start_interfaces) {
    for (size_t i = 0; i < RMDJoints_.size(); ++i) {
      auto & joint = RMDJoints_[i];
      const std::string pos_if = joint.name + "/" + std::string(hardware_interface::HW_IF_POSITION);
      const std::string vel_if = joint.name + "/" + std::string(hardware_interface::HW_IF_VELOCITY);
      if (ifname == pos_if) {
        requested_modes[i] = integration_level_t::POSITION;
      } else if (ifname == vel_if) {
        requested_modes[i] = integration_level_t::VELOCITY;
      }
    }
  }

  // Now apply the requested_modes to each joint.
  // For any joint with UNDEFINED in requested_modes, we only change it if it was explicitly stopped.
  for (size_t i = 0; i < RMDJoints_.size(); ++i) {
    auto & joint = RMDJoints_[i];
    if (requested_modes[i] == integration_level_t::UNDEFINED) {
      // if stop requested, set to UNDEFINED; otherwise leave existing mode
      // (we only set to UNDEFINED if this joint was mentioned in stop_interfaces)
      bool was_stopped = false;
      for (const auto &ifname : stop_interfaces) {
        if (ifname.find(joint.name) != std::string::npos) {
          was_stopped = true;
          break;
        }
      }
      if (was_stopped) {
        joint.control_level = integration_level_t::UNDEFINED;
        joint.joint_command_velocity = 0;
        // optional: reset position cmd to current state to be safe
        joint.joint_command_position = joint.joint_state_position;
        RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"),
          "Joint %s: stopped -> set UNDEFINED", joint.name.c_str());
      }
      // else, leave the existing control mode as-is
    } else {
      // Set the mode requested
      joint.control_level = requested_modes[i];
      // If switching to velocity, optionally set command velocity to current state to avoid jumps
      if (requested_modes[i] == integration_level_t::VELOCITY) {
        joint.joint_command_velocity = 0;
        RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"),
          "Joint %s: switched to VELOCITY (cmd vel initialized from state: %f)",
          joint.name.c_str(), joint.joint_command_velocity);
      } else if (requested_modes[i] == integration_level_t::POSITION) {
        joint.joint_command_position = joint.joint_state_position;
        RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"),
          "Joint %s: switched to POSITION (cmd pos initialized from state: %f)",
          joint.name.c_str(), joint.joint_command_position);
      }
    }
  }

  return hardware_interface::return_type::OK;
}


}  // namespace rmd_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rmd_ros2_control::RMDHardwareInterface, hardware_interface::SystemInterface)

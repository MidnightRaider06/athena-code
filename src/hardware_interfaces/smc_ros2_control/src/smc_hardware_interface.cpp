#include "smc_ros2_control/smc_hardware_interface.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace smc_ros2_control
{

// -- HELPER FUNCTIONS -- //

double SMCHardwareInterface::calculate_joint_position_from_motor_position(
  double motor_position, int gear_ratio)
{
  return (motor_position * 0.01 * (M_PI / 180.0)) / gear_ratio;
}

double SMCHardwareInterface::calculate_joint_velocity_from_motor_velocity(
  double motor_velocity, int gear_ratio)
{
  return (motor_velocity * (M_PI / 180.0)) / gear_ratio;
}

int32_t SMCHardwareInterface::calculate_motor_position_from_desired_joint_position(
  double joint_position, int gear_ratio)
{
  return static_cast<int32_t>(std::round((joint_position * (180.0 / M_PI) * 100.0) * gear_ratio));
}

int32_t SMCHardwareInterface::calculate_motor_velocity_from_desired_joint_velocity(
  double joint_velocity, int gear_ratio)
{
  return static_cast<int32_t>(std::round((joint_velocity * (180.0 / M_PI) * 100.0) * gear_ratio));
}

void SMCHardwareInterface::send_command(int can_id, int cmd_id)
{
  CANLib::CanFrame frame;
  frame.id = can_id;
  frame.dlc = 8;
  frame.data.fill(0x00);
  frame.data[0] = static_cast<uint8_t>(cmd_id);
  canBus.send(frame);
}

void SMCHardwareInterface::logger_function()
{
  // Prevent breaking the logger
  if (SMCJoints_.empty()) {
    return;
  }

  std::string log_msg = "\033[2J\033[H \nSMC Logger";
  std::string control_mode = "";

  std::ostringstream oss;
  oss << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface
      << " | HWI Update Rate: " << update_rate
      << " | Logger Update Rate: " << logger_rate << "\n"
      << "Elapsed Time since first update: " << elapsed_time << "\n"
      << "\n--- Joint Specific ---";

  for (auto & joint : SMCJoints_) {

    if (joint.control_level == integration_level_t::POSITION) {
      control_mode = "POSITION";
    } else if (joint.control_level == integration_level_t::VELOCITY) {
      control_mode = "VELOCITY";
    } else {
      control_mode = "UNDEFINED";
    }

    oss << "\n----- JOINT: " << joint.name << " -----\n"
        << "Parameters: CAN ID: 0x"
        << std::hex << std::uppercase << joint.node_id << std::dec
        << " | Gear Ratio: " << joint.gear_ratio
        << " | Orientation: " << joint.orientation
        << " | Operating Velocity: " << joint.operating_velocity << "\n"

        << "Curr P: " << joint.current_Kp
        << " | Curr I: " << joint.current_Ki
        << " | Curr D: " << joint.current_Kd << "\n"

        << "Speed P: " << joint.speed_Kp
        << " | Speed I: " << joint.speed_Ki
        << " | Speed D: " << joint.speed_Kd << "\n"

        << "Pos P: " << joint.position_Kp
        << " | Pos I: " << joint.position_Ki
        << " | Pos D: " << joint.position_Kd << "\n"

        << "Max Torque: " << joint.max_torque
        << " | Max Speed: " << joint.max_speed
        << " | Max Angle: " << joint.max_angle << "\n"

        << "Current Ramp: " << joint.current_ramp
        << " | Speed Ramp: " << joint.speed_ramp << "\n"

        << "\n-- Commands --\n"
        << "Control Mode: " << control_mode << "\n"
        << "Motor Position: " << joint.motor_position
        << " | Joint Command Position: " << joint.joint_command_position << "\n"

        << "Motor Velocity: " << joint.motor_velocity
        << " | Joint Command Velocity: " << joint.joint_command_velocity << "\n"

        << "Motor Status Request: " << joint.motor_status_req
        << " | Motor Maintenance Request: " << joint.motor_maintenance_req << "\n"

        << "Maintenance Command High: " << joint.maintenance_frame_high
        << " | Low: " << joint.maintenance_frame_low
        << " | Full: " << joint.maintenance_frame << "\n"

        << "Maintenance Data Count: " << joint.maintenance_data_count << "\n"

        << "Previous Status Req: " << joint.prev_status_req
        << " | Previous Maintenance Req: " << joint.prev_maintenance_req << "\n"

        << "Elapsed Status Req Time: " << joint.elapsed_status_request_time
        << " | Elapsed Maintenance Req Time: "
        << joint.elapsed_maintenance_request_time << "\n"

        << "Decoded Maintenance Frame: ";

    for (const auto & byte : joint.decoded_maintenance_frame) {
      oss << std::hex << std::uppercase
          << static_cast<int>(byte) << " ";
    }

    oss << std::dec
        << "\n-- State --\n"
        << "Joint Position: " << joint.joint_state_position << " rad"
        << " | Joint Velocity: " << joint.joint_state_velocity << " rad/s"
        << " | Acceleration: " << joint.acceleration << "\n"

        << "Encoder Position: " << joint.encoder_position << "\n"

        << "\n-- Telemetry --\n"
        << "Motor Temperature: " << joint.motor_temperature << " C"
        << " | Torque Current: " << joint.motor_torque_current << " A"
        << " | Motor Status: " << joint.motor_status << "\n"

        << "\n-- Previous Commands --\n"
        << "Previous Joint Command Position: "
        << joint.prev_joint_command_position
        << " | Previous Joint Command Velocity: "
        << joint.prev_joint_command_velocity << "\n";

    log_msg += oss.str();
    RCLCPP_INFO(
      rclcpp::get_logger("SMCHardwareInterface"),
      log_msg.c_str());

    oss.str("");
    oss.clear();
  }
}

bool SMCHardwareInterface::interpret_settings_parameters(
  SMCJoint & joint, const std::array<uint8_t, 8> & data)
{
  const auto param = static_cast<ParamID>(data[1]);
  switch (param) {
    case ParamID::ANGLE_PID:
      joint.position_Kp = static_cast<double>(static_cast<int16_t>((data[3] << 8) | data[2]));
      joint.position_Ki = static_cast<double>(static_cast<int16_t>((data[5] << 8) | data[4]));
      joint.position_Kd = static_cast<double>(static_cast<int16_t>((data[7] << 8) | data[6]));
      return true;
    case ParamID::SPEED_PID:
      joint.speed_Kp = static_cast<double>(static_cast<int16_t>((data[3] << 8) | data[2]));
      joint.speed_Ki = static_cast<double>(static_cast<int16_t>((data[5] << 8) | data[4]));
      joint.speed_Kd = static_cast<double>(static_cast<int16_t>((data[7] << 8) | data[6]));
      return true;
    case ParamID::CURRENT_PID:
      joint.current_Kp = static_cast<double>(static_cast<int16_t>((data[3] << 8) | data[2]));
      joint.current_Ki = static_cast<double>(static_cast<int16_t>((data[5] << 8) | data[4]));
      joint.current_Kd = static_cast<double>(static_cast<int16_t>((data[7] << 8) | data[6]));
      return true;
    case ParamID::MAX_TORQUE:
      joint.max_torque = static_cast<double>(static_cast<int16_t>((data[3] << 8) | data[2]));
      return true;
    case ParamID::MAX_SPEED:
      joint.max_speed = static_cast<double>(static_cast<int32_t>((data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2]));
      return true;
    case ParamID::MAX_ANGLE_A:
    case ParamID::MAX_ANGLE_B:
      joint.max_angle = static_cast<double>(static_cast<int32_t>((data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2])) * 0.01 * (M_PI / 180.0);
      return true;
    case ParamID::CURRENT_RAMP:
      joint.current_ramp = static_cast<double>(static_cast<int16_t>((data[3] << 8) | data[2]));
      return true;
    case ParamID::SPEED_RAMP:
      joint.speed_ramp = static_cast<double>(static_cast<int32_t>((data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2]));
      return true;
    default:
      return false;
  }
}

void SMCHardwareInterface::format_control_command(CANLib::CanFrame & frame, SMCJoint & joint)
{
  std::fill(std::begin(frame.data), std::end(frame.data), 0x00);

  if (
    joint.control_level == integration_level_t::POSITION &&
    std::isfinite(joint.joint_command_position) &&
    joint.joint_command_position != joint.prev_joint_command_position)
  {
    const int32_t joint_angle = joint.orientation *
      calculate_motor_position_from_desired_joint_position(
        joint.joint_command_position, joint.gear_ratio);

    frame.data[0] = static_cast<uint8_t>(ControlCommands::ABSOLUTE_POS_WITH_VEL_CONTROL_CMD);
    frame.data[2] = static_cast<uint8_t>(joint.operating_velocity & 0xFF);
    frame.data[3] = static_cast<uint8_t>((joint.operating_velocity >> 8) & 0xFF);
    frame.data[4] = static_cast<uint8_t>(joint_angle & 0xFF);
    frame.data[5] = static_cast<uint8_t>((joint_angle >> 8) & 0xFF);
    frame.data[6] = static_cast<uint8_t>((joint_angle >> 16) & 0xFF);
    frame.data[7] = static_cast<uint8_t>((joint_angle >> 24) & 0xFF);
    joint.prev_joint_command_position = joint.joint_command_position;
    return;
  }
  else if (
    joint.control_level == integration_level_t::VELOCITY &&
    std::isfinite(joint.joint_command_velocity) &&
    joint.joint_command_velocity != joint.prev_joint_command_velocity)
  {
    const int32_t joint_velocity = joint.orientation *
      calculate_motor_velocity_from_desired_joint_velocity(
        joint.joint_command_velocity, joint.gear_ratio);

    frame.data[0] = static_cast<uint8_t>(ControlCommands::SPEED_CONTROL_CMD);
    frame.data[4] = static_cast<uint8_t>(joint_velocity & 0xFF);
    frame.data[5] = static_cast<uint8_t>((joint_velocity >> 8) & 0xFF);
    frame.data[6] = static_cast<uint8_t>((joint_velocity >> 16) & 0xFF);
    frame.data[7] = static_cast<uint8_t>((joint_velocity >> 24) & 0xFF);
    joint.prev_joint_command_velocity = joint.joint_command_velocity;
    return;
  }
  else{
    state_iterator = state_iterator + 1;
    (state_iterator%2 == 0) ? frame.data[0] = static_cast<uint8_t>(StatusCommands::MOTOR_STATUS_2_CMD) :
                         frame.data[0] = static_cast<uint8_t>(StatusCommands::READ_ABS_ANGLE_CMD);
    return;
  }
}

bool SMCHardwareInterface::format_maintenance_command(
  CANLib::CanFrame & frame, const DecodedCommand & decoded_cmd)
{
  std::fill(std::begin(frame.data), std::end(frame.data), 0x00);
  frame.data[0] = decoded_cmd.command_id; // Set multiplexor
  switch (static_cast<MaintenanceCommands>(decoded_cmd.command_id)) {
    case MaintenanceCommands::WRITE_ACCELERATION_CMD:
      if(decoded_cmd.u8_data.size() != 0 || decoded_cmd.i16_data.size() != 0 || decoded_cmd.i32_data.size() != 1){
        return false; // Invalid data format for this command
      }
      else{
        frame.data[4] = decoded_cmd.i32_data[0] & 0xFF; // Acceleration low byte
        frame.data[5] = (decoded_cmd.i32_data[0] >> 8) & 0xFF; // Acceleration byte 2
        frame.data[6] = (decoded_cmd.i32_data[0] >> 16) & 0xFF; // Acceleration byte 3
        frame.data[7] = (decoded_cmd.i32_data[0] >> 24) & 0xFF; // Acceleration high byte
        return true;
      }
    case MaintenanceCommands::READ_SETTINGS_CMD:
      if(decoded_cmd.u8_data.size() != 1 || decoded_cmd.i16_data.size() != 0 || decoded_cmd.i32_data.size() != 0){
        return false; // Invalid data format for this command
      }
      else{
        frame.data[1] = decoded_cmd.u8_data[0]; // Parameter ID
        return true;
      }
    case MaintenanceCommands::WRITE_SETTINGS_TO_RAM_CMD:
      if(decoded_cmd.u8_data.size() != 1 || decoded_cmd.i16_data.size() != 0 || decoded_cmd.i32_data.size() != 0){
        return false; // Invalid data format for this command
      }
      else{
        frame.data[1] = decoded_cmd.u8_data[0]; // Parameter ID
        return true;
      }
    case MaintenanceCommands::WRITE_SETTINGS_TO_ROM_CMD:
      if(decoded_cmd.u8_data.size() != 1 || decoded_cmd.i16_data.size() != 0 || decoded_cmd.i32_data.size() != 0){
        return false; // Invalid data format for this command
      }
      else{
        frame.data[1] = decoded_cmd.u8_data[0]; // Parameter ID
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
    case MaintenanceCommands::MOTOR_RUNNING_CMD:
      if(decoded_cmd.u8_data.size() != 0 || decoded_cmd.i16_data.size() != 0 || decoded_cmd.i32_data.size() != 0){
        return false; // Invalid data format for this command
      }
      else{
        // Empty
        return true;
      }
    case MaintenanceCommands::CLEAR_MOTOR_ANGLE_CMD:
      if(decoded_cmd.u8_data.size() != 0 || decoded_cmd.i16_data.size() != 0 || decoded_cmd.i32_data.size() != 0){
        return false; // Invalid data format for this command
      }
      else{
        // Empty
        return true;
      }
    case MaintenanceCommands::CLEAR_ERROR_CMD:
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

  size_t index = 1;
  for (uint8_t value : decoded_cmd.u8_data) {
    if (index >= frame.data.size()) {
      return false;
    }
    frame.data[index++] = value;
  }
  for (int16_t value : decoded_cmd.i16_data) {
    if (index + 1 >= frame.data.size()) {
      return false;
    }
    frame.data[index++] = static_cast<uint8_t>(value & 0xFF);
    frame.data[index++] = static_cast<uint8_t>((value >> 8) & 0xFF);
  }
  for (int32_t value : decoded_cmd.i32_data) {
    if (index + 3 >= frame.data.size()) {
      return false;
    }
    frame.data[index++] = static_cast<uint8_t>(value & 0xFF);
    frame.data[index++] = static_cast<uint8_t>((value >> 8) & 0xFF);
    frame.data[index++] = static_cast<uint8_t>((value >> 16) & 0xFF);
    frame.data[index++] = static_cast<uint8_t>((value >> 24) & 0xFF);
  }
  return true;
}

hardware_interface::CallbackReturn SMCHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  SMCJoints_.clear();
  update_rate = std::stoi(info_.hardware_parameters.at("update_rate"));
  logger_rate = std::stoi(info_.hardware_parameters.at("logger_rate"));
  logger_state = std::stoi(info_.hardware_parameters.at("logger_state"));
  can_interface = info_.hardware_parameters.at("can_interface");

  // Per Joint Parameters
  for (const auto & joint : info_.joints) {
    // Collect joint specific parameters
    const int node_id = std::clamp(std::stoi(joint.parameters.at("node_id"), nullptr, 0), 0x141, 0x160);
    const int gear_ratio = std::abs(std::stoi(joint.parameters.at("gear_ratio")));
    const int orientation = joint.parameters.count("joint_orientation") &&
      std::stoi(joint.parameters.at("joint_orientation")) == -1 ? -1 : 1;
    const int operating_velocity = std::clamp(
      std::stoi(joint.parameters.at("operating_velocity")), 0, 65 * gear_ratio);
    
    // Collect state interface names
    std::vector<std::string> state_if_names;
    for (const auto & si : joint.state_interfaces) {
      state_if_names.push_back(si.name);
    }
    
    // Collect command interface names
    std::vector<std::string> command_if_names;
    for (const auto & ci : joint.command_interfaces) {
      command_if_names.push_back(ci.name);
    }
    
    // Copy parameters into an unordered_map<string,string>
    std::unordered_map<std::string, std::string> params_map;
    for (const auto & p : joint.parameters) {
      params_map.emplace(p.first, p.second);
    }

    // Populate each SMCJoint object
    SMCJoint smc_joint{};
    SMCJoints_.push_back(
      SMCJoint{
        .name = joint.name,
        .node_id = static_cast<uint32_t>(node_id),
        .gear_ratio = gear_ratio,
        .orientation = orientation,
        .operating_velocity = static_cast<uint16_t>(operating_velocity),
        .control_level = integration_level_t::POSITION,
        .joint_state_position = std::numeric_limits<double>::quiet_NaN(),
        .joint_state_velocity = 0.0,
        .motor_temperature = std::numeric_limits<double>::quiet_NaN(),
        .motor_torque_current = std::numeric_limits<double>::quiet_NaN(),
        .motor_status = 0.0,
        .current_Kp = std::numeric_limits<double>::quiet_NaN(),
        .current_Ki = std::numeric_limits<double>::quiet_NaN(),
        .current_Kd = std::numeric_limits<double>::quiet_NaN(),
        .speed_Kp = std::numeric_limits<double>::quiet_NaN(),
        .speed_Ki = std::numeric_limits<double>::quiet_NaN(),
        .speed_Kd = std::numeric_limits<double>::quiet_NaN(),
        .position_Kp = std::numeric_limits<double>::quiet_NaN(),
        .position_Ki = std::numeric_limits<double>::quiet_NaN(),
        .position_Kd = std::numeric_limits<double>::quiet_NaN(),
        .acceleration = std::numeric_limits<double>::quiet_NaN(),
        .max_torque = std::numeric_limits<double>::quiet_NaN(),
        .max_speed = std::numeric_limits<double>::quiet_NaN(),
        .max_angle = std::numeric_limits<double>::quiet_NaN(),
        .current_ramp = std::numeric_limits<double>::quiet_NaN(),
        .speed_ramp = std::numeric_limits<double>::quiet_NaN(),

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
        .encoder_position = 0.0,
        .prev_joint_command_position = std::numeric_limits<double>::quiet_NaN(),
        .prev_joint_command_velocity = std::numeric_limits<double>::quiet_NaN(),
        .state_interface_names = state_if_names,
        .command_interface_names = command_if_names,
        .parameters = params_map
      }
    );
  }

  num_joints = static_cast<int>(SMCJoints_.size());
  state_iterator = 0;
  elapsed_update_time = 0.0;
  elapsed_time = 0.0;
  elapsed_logger_time = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SMCHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

std::vector<hardware_interface::StateInterface> SMCHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & joint : SMCJoints_) {
    for (const auto & iface : joint.state_interface_names) {
      double * value = nullptr;
      if (iface == hardware_interface::HW_IF_POSITION) {
        value = &joint.joint_state_position;
      } else if (iface == hardware_interface::HW_IF_VELOCITY) {
        value = &joint.joint_state_velocity;
      } else if (iface == "motor_temperature") {
        value = &joint.motor_temperature;
      } else if (iface == "torque_current") {
        value = &joint.motor_torque_current;
      } else if (iface == "status") {
        value = &joint.motor_status;
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("SMCHardwareInterface"),
          "Unknown state interface '%s' for joint '%s'",
          iface.c_str(), joint.name.c_str());
        continue;
      }
      state_interfaces.emplace_back(joint.name, iface, value);
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SMCHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto & joint : SMCJoints_) {
    for (const auto & iface : joint.command_interface_names) {
      double * value = nullptr;
      if (iface == hardware_interface::HW_IF_POSITION) {
        value = &joint.joint_command_position;
      } else if (iface == hardware_interface::HW_IF_VELOCITY) {
        value = &joint.joint_command_velocity;
      } else if (iface == "status_request") {
        value = &joint.motor_status_req;
      } else if (iface == "maintenance_request") {
        value = &joint.motor_maintenance_req;
      } else if (iface == "maintenance_frame_high") {
        value = &joint.maintenance_frame_high;
      } else if (iface == "maintenance_frame_low") {
        value = &joint.maintenance_frame_low;
      } else if (iface == "maintenance_data_count") {
        value = &joint.maintenance_data_count;
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("SMCHardwareInterface"),
          "Unknown command interface '%s' for joint '%s'",
          iface.c_str(), joint.name.c_str());
        continue;
      }
      command_interfaces.emplace_back(joint.name, iface, value);
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn SMCHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!canBus.open(
      can_interface,
      std::bind(&SMCHardwareInterface::on_can_message, this, std::placeholders::_1)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("SMCHardwareInterface"), "Failed to open CAN interface");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

void SMCHardwareInterface::on_can_message(const CANLib::CanFrame & frame)
{
  can_rx_frame_ = frame;
  for (auto & joint : SMCJoints_) {
    if (frame.id != joint.node_id) {
      continue;
    }

    if (
      frame.data[0] == static_cast<uint8_t>(ControlCommands::SPEED_CONTROL_CMD) ||
      frame.data[0] == static_cast<uint8_t>(ControlCommands::ABSOLUTE_POS_WITH_VEL_CONTROL_CMD) ||
      frame.data[0] == static_cast<uint8_t>(StatusCommands::MOTOR_STATUS_2_CMD))
    {
      joint.encoder_position = static_cast<double>(
        static_cast<int16_t>((frame.data[7] << 8) | frame.data[6]));
      joint.motor_velocity = static_cast<double>(
        static_cast<int16_t>((frame.data[5] << 8) | frame.data[4]));
      joint.motor_temperature = static_cast<double>(frame.data[1]);
      joint.motor_torque_current = static_cast<double>(
        static_cast<int16_t>((frame.data[3] << 8) | frame.data[2])) * 0.01;
      continue;
    }

    if (frame.data[0] == static_cast<uint8_t>(StatusCommands::READ_ABS_ANGLE_CMD)) {
      const uint64_t raw =
        (static_cast<uint64_t>(frame.data[7]) << 48) |
        (static_cast<uint64_t>(frame.data[6]) << 40) |
        (static_cast<uint64_t>(frame.data[5]) << 32) |
        (static_cast<uint64_t>(frame.data[4]) << 24) |
        (static_cast<uint64_t>(frame.data[3]) << 16) |
        (static_cast<uint64_t>(frame.data[2]) << 8) |
        static_cast<uint64_t>(frame.data[1]);
      joint.motor_position = static_cast<double>((static_cast<int64_t>(raw) << 8) >> 8);
      continue;
    }

    if (frame.data[0] == static_cast<uint8_t>(StatusCommands::MOTOR_STATUS_1_CMD)) {
      joint.motor_temperature = static_cast<double>(frame.data[1]);
      joint.motor_status = static_cast<double>(frame.data[7]);
      continue;
    }

    if (frame.data[0] == static_cast<uint8_t>(StatusCommands::READ_ACCELERATION_CMD)) {
      joint.acceleration = static_cast<double>(
        (frame.data[7] << 24) | (frame.data[6] << 16) | (frame.data[5] << 8) | frame.data[4]);
    }

    if(frame.data[0] == static_cast<uint8_t>(MaintenanceCommands::READ_SETTINGS_CMD)) {
      if(!interpret_settings_parameters(joint, frame.data)) {
        RCLCPP_ERROR(rclcpp::get_logger("SMCHardwareInterface"), "Failed to interpret settings parameters for joint '%s'.", joint.name.c_str());
      }
    }
  }
}

hardware_interface::CallbackReturn SMCHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  for (const auto & joint : SMCJoints_) {
    send_command(joint.node_id, static_cast<int>(MaintenanceCommands::MOTOR_SHUTDOWN_CMD));
  }
  canBus.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SMCHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  for (const auto & joint : SMCJoints_) {
    send_command(joint.node_id, static_cast<int>(MaintenanceCommands::MOTOR_RUNNING_CMD));
  }
  for (auto & joint : SMCJoints_) {
    joint.joint_command_position = joint.joint_state_position;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SMCHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (const auto & joint : SMCJoints_) {
    send_command(joint.node_id, static_cast<int>(MaintenanceCommands::MOTOR_STOP_CMD));
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SMCHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto & joint : SMCJoints_) {
    joint.joint_state_velocity = calculate_joint_velocity_from_motor_velocity(
      joint.motor_velocity, joint.gear_ratio);
    joint.joint_state_position = calculate_joint_position_from_motor_position(
      joint.motor_position, joint.gear_ratio);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SMCHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  // Logger update
  elapsed_time += period.seconds();
  elapsed_logger_time += period.seconds();
  if (logger_rate > 0 && elapsed_logger_time > (1.0 / static_cast<double>(logger_rate))) {
    elapsed_logger_time = 0.0;
    if (logger_state == 1) {
      logger_function();
    }
  }

  // Status request handling
  for (auto & joint : SMCJoints_) {
    const double curr_status_req = joint.motor_status_req;
    if (curr_status_req < 0.0 && joint.prev_status_req >= 0.0) 
    {
      for (auto status_cmd : kStatusCommands) {
        send_command(joint.node_id, static_cast<int>(status_cmd));
      }
      RCLCPP_INFO(rclcpp::get_logger("SMCHardwareInterface"), "One-shot status request sent for joint '%s'.", joint.name.c_str());
    } 
    else if (curr_status_req > 0.0) 
    {
      joint.elapsed_status_request_time += period.seconds();
      double status_request_period = 1.0 / curr_status_req;
      if (joint.elapsed_status_request_time > status_request_period) {
        joint.elapsed_status_request_time = 0.0;
        for (auto status_cmd : kStatusCommands) {
          send_command(joint.node_id, static_cast<int>(status_cmd));
        }
      }
    }
    joint.prev_status_req = curr_status_req;
  }

  // Maintenance request handling
  for (auto & joint : SMCJoints_) {
    auto doubles_to_payload = [](double high, double low) -> int64_t {
      const uint64_t h = static_cast<uint64_t>(high);
      const uint64_t l = static_cast<uint64_t>(low);
      return static_cast<int64_t>((h << 32) | l);
    };

    joint.maintenance_frame = static_cast<double>(doubles_to_payload(
      joint.maintenance_frame_high, joint.maintenance_frame_low));
    const auto decoded_maintenance_cmd = unpack_command_full(
      static_cast<int32_t>(joint.maintenance_data_count),
      static_cast<int64_t>(joint.maintenance_frame));
    pack_decoded_maintenance_frame(joint, decoded_maintenance_cmd);

    CANLib::CanFrame frame;
    frame.id = joint.node_id;
    frame.dlc = 8;
    if (!format_maintenance_command(frame, decoded_maintenance_cmd)) {
      // RCLCPP_WARN(rclcpp::get_logger("SMCHardwareInterface"), "Invalid maintenance command for joint '%s'.", joint.name.c_str());
      continue;
    }

    
    const double curr_maintenance_req = joint.motor_maintenance_req;
    if (curr_maintenance_req < 0.0 && joint.prev_maintenance_req >= 0.0) { // One-shot maintenance command
      canBus.send(frame);
      RCLCPP_INFO(rclcpp::get_logger("SMCHardwareInterface"), "One-shot maintenance request sent for joint '%s'.", joint.name.c_str());
    } else if (curr_maintenance_req > 0.0) {
      joint.elapsed_maintenance_request_time += period.seconds();
      double maintenance_request_period = 1.0 / curr_maintenance_req;
      if (joint.elapsed_maintenance_request_time > maintenance_request_period) {
        joint.elapsed_maintenance_request_time = 0.0;
        canBus.send(frame);
        RCLCPP_INFO(rclcpp::get_logger("SMCHardwareInterface"), "Periodic maintenance request sent for joint '%s'.", joint.name.c_str());
      }
    }
    joint.prev_maintenance_req = curr_maintenance_req;
  }

  // HWI can only go as fast as the controller manager. To limit frequency of bus messages,
  // keep track of time passed over iterations of this function and if it exceeds the 
  // desired frequency of the HWI, skip message
  elapsed_update_time += period.seconds();
  if (update_rate > 0 && elapsed_update_time > (1.0 / static_cast<double>(update_rate))) {
    elapsed_update_time = 0.0;
    for (auto & joint : SMCJoints_) {
      CANLib::CanFrame frame;
      frame.id = joint.node_id;
      frame.dlc = 8;
      format_control_command(frame, joint);
      canBus.send(frame);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SMCHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  std::vector<integration_level_t> requested_modes(
    static_cast<size_t>(num_joints), integration_level_t::UNDEFINED);

  for (const auto & ifname : stop_interfaces) {
    for (size_t i = 0; i < SMCJoints_.size(); ++i) {
      const auto & joint = SMCJoints_[i];
      const std::string pos_if = joint.name + "/" + hardware_interface::HW_IF_POSITION;
      const std::string vel_if = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
      if (ifname == pos_if || ifname == vel_if || ifname.find(joint.name) != std::string::npos) {
        requested_modes[i] = integration_level_t::UNDEFINED;
      }
    }
  }

  for (const auto & ifname : start_interfaces) {
    for (size_t i = 0; i < SMCJoints_.size(); ++i) {
      const auto & joint = SMCJoints_[i];
      const std::string pos_if = joint.name + "/" + hardware_interface::HW_IF_POSITION;
      const std::string vel_if = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
      if (ifname == pos_if) {
        requested_modes[i] = integration_level_t::POSITION;
      } else if (ifname == vel_if) {
        requested_modes[i] = integration_level_t::VELOCITY;
      }
    }
  }

  for (size_t i = 0; i < SMCJoints_.size(); ++i) {
    auto & joint = SMCJoints_[i];
    if (requested_modes[i] == integration_level_t::UNDEFINED) {
      bool was_stopped = false;
      for (const auto & ifname : stop_interfaces) {
        if (ifname.find(joint.name) != std::string::npos) {
          was_stopped = true;
          break;
        }
      }
      if (was_stopped) {
        joint.control_level = integration_level_t::UNDEFINED;
        joint.joint_command_velocity = 0.0;
        joint.joint_command_position = joint.joint_state_position;
      }
      continue;
    }

    joint.control_level = requested_modes[i];
    if (joint.control_level == integration_level_t::VELOCITY) {
      joint.joint_command_velocity = 0.0;
    } else if (joint.control_level == integration_level_t::POSITION) {
      joint.joint_command_position = joint.joint_state_position;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace smc_ros2_control

PLUGINLIB_EXPORT_CLASS(
  smc_ros2_control::SMCHardwareInterface, hardware_interface::SystemInterface)

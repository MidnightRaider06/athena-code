#include "dc_ros2_control/dc_hardware_interface.hpp"

#include <netdb.h>
#include <sys/socket.h>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <algorithm>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dc_ros2_control
{

// =============================================================================
// Unit Conversions (matches servo HWI structure)
// =============================================================================

double DCHardwareInterface::calculate_joint_position_from_motor_position(double motor_pos_deg, int gear_ratio) {
  // deg -> radians with gear ratio
  return (motor_pos_deg * (M_PI / 180.0)) / gear_ratio;
}

double DCHardwareInterface::calculate_joint_displacement_from_motor_position(double motor_pos_deg, int gear_ratio, double dist_per_rev) {
  // deg -> meters: (deg / 360) * distance_per_rev / gear_ratio
  return (motor_pos_deg / 360.0) * dist_per_rev / gear_ratio;
}

double DCHardwareInterface::calculate_joint_angular_velocity_from_motor_velocity(double motor_vel_dps, int gear_ratio) {
  // deg/s -> rad/s with gear ratio
  return (motor_vel_dps * (M_PI / 180.0)) / gear_ratio;
}

double DCHardwareInterface::calculate_joint_linear_velocity_from_motor_velocity(double motor_vel_dps, int gear_ratio, double dist_per_rev) {
  // deg/s -> m/s
  return (motor_vel_dps / 360.0) * dist_per_rev / gear_ratio;
}

int16_t DCHardwareInterface::calculate_motor_position_from_desired_joint_position(double joint_position, int gear_ratio) {
  // radians -> deg with gear ratio
  return static_cast<int16_t>(std::round((joint_position * (180.0 / M_PI)) * gear_ratio));
}

int16_t DCHardwareInterface::calculate_motor_position_from_desired_joint_displacement(double joint_position, int gear_ratio, double dist_per_rev) {
  // meters -> deg: (m / distance_per_rev) * 360 * gear_ratio
  return static_cast<int16_t>(std::round((joint_position / dist_per_rev) * 360.0 * gear_ratio));
}

int16_t DCHardwareInterface::calculate_motor_velocity_from_desired_joint_angular_velocity(double joint_velocity, int gear_ratio) {
  // rad/s -> deg/s with gear ratio
  return static_cast<int16_t>(std::round((joint_velocity * (180.0 / M_PI)) * gear_ratio));
}

int16_t DCHardwareInterface::calculate_motor_velocity_from_desired_joint_linear_velocity(double joint_velocity, int gear_ratio, double dist_per_rev) {
  // m/s -> deg/s
  return static_cast<int16_t>(std::round((joint_velocity / dist_per_rev) * 360.0 * gear_ratio));
}

void DCHardwareInterface::format_control_command(CANLib::CanFrame & frame, size_t joint_index)
{
  std::fill(std::begin(frame.data), std::end(frame.data), 0x00);

  auto & joint = DCJoints_[joint_index];
  const double dir = joint.inverted ? -1.0 : 1.0;

  if (
    joint.control_level == integration_level_t::POSITION &&
    std::isfinite(joint.joint_command_position) &&
    joint.joint_command_position != joint.prev_joint_command_position)
  {
    joint.prev_joint_command_position = joint.joint_command_position;
    if (joint.joint_type == joint_type_t::PRISMATIC) {
      joint.joint_command_position = std::clamp(
        joint.joint_command_position, 0.0, joint.max_disp);
    }

    int16_t motor_pos = 0;
    if (joint.joint_type == joint_type_t::REVOLUTE) {
      motor_pos = calculate_motor_position_from_desired_joint_position(
        dir * joint.joint_command_position, joint.gear_ratio);
    } else {
      motor_pos = calculate_motor_position_from_desired_joint_displacement(
        dir * joint.joint_command_position, joint.gear_ratio, joint.distance_per_rev);
    }

    frame.dlc = 3;
    frame.data[0] = static_cast<uint8_t>(
      static_cast<uint8_t>(ControlCommands::ABSOLUTE_POS_CONTROL_CMD) + joint.node_id);
    frame.data[1] = static_cast<uint8_t>(motor_pos & 0xFF);
    frame.data[2] = static_cast<uint8_t>((motor_pos >> 8) & 0xFF);
    return;
  }

  if (
    joint.control_level == integration_level_t::VELOCITY &&
    std::isfinite(joint.joint_command_velocity) &&
    joint.joint_command_velocity != joint.prev_joint_command_velocity)
  {
    joint.prev_joint_command_velocity = joint.joint_command_velocity;
    int16_t motor_vel = 0;
    if (joint.joint_type == joint_type_t::REVOLUTE) {
      motor_vel = calculate_motor_velocity_from_desired_joint_angular_velocity(
        dir * joint.joint_command_velocity, joint.gear_ratio);
    } else {
      motor_vel = calculate_motor_velocity_from_desired_joint_linear_velocity(
        dir * joint.joint_command_velocity, joint.gear_ratio, joint.distance_per_rev);
    }

    frame.dlc = 3;
    frame.data[0] = static_cast<uint8_t>(
      static_cast<uint8_t>(ControlCommands::VELOCITY_CONTROL_CMD) + joint.node_id);
    frame.data[1] = static_cast<uint8_t>(motor_vel & 0xFF);
    frame.data[2] = static_cast<uint8_t>((motor_vel >> 8) & 0xFF);
    return;
  }

  frame.dlc = 2;
  frame.data[0] = static_cast<uint8_t>(
    static_cast<uint8_t>(StatusCommands::MOTOR_STATE) + joint.node_id);
  frame.data[1] = static_cast<uint8_t>(ValidateRequest::VALID);
}

bool DCHardwareInterface::format_status_command(
  CANLib::CanFrame & frame, uint8_t command_id, uint8_t node_id)
{
  std::fill(std::begin(frame.data), std::end(frame.data), 0x00);
  frame.dlc = 2;
  switch (static_cast<StatusCommands>(command_id)) {
    case StatusCommands::MOTOR_STATE:
      frame.data[0] = static_cast<uint8_t>(static_cast<uint8_t>(StatusCommands::MOTOR_STATE) + node_id);
      frame.data[1] = static_cast<uint8_t>(ValidateRequest::VALID);
      return true;
    case StatusCommands::MOTOR_STATUS:
      frame.data[0] = static_cast<uint8_t>(static_cast<uint8_t>(StatusCommands::MOTOR_STATUS) + node_id);
      frame.data[1] = static_cast<uint8_t>(ValidateRequest::VALID);
      return true;
    default:
      return false;
  }
}

bool DCHardwareInterface::format_maintenance_command(
  CANLib::CanFrame & frame, uint8_t node_id, const DecodedCommand & decoded_cmd)
{
  std::fill(std::begin(frame.data), std::end(frame.data), 0x00);
  frame.dlc = 2;
  frame.data[0] = static_cast<uint8_t>(
    static_cast<uint8_t>(MaintenanceCommands::MAINTENANCE_CMD) + node_id);
  frame.data[1] = decoded_cmd.command_id;

  switch (static_cast<MaintenanceCommands>(decoded_cmd.command_id)) {
    case MaintenanceCommands::PCB_HEARTBEAT_CMD:
      if (decoded_cmd.u8_data.size() != 1 || !decoded_cmd.i16_data.empty() ||
        !decoded_cmd.i32_data.empty())
      {
        return false;
      }
      frame.data[0] = static_cast<uint8_t>(MaintenanceCommands::PCB_HEARTBEAT_CMD);
      frame.data[1] = decoded_cmd.u8_data[0];
      return true;
    case MaintenanceCommands::MAINTENANCE_CMD:
      if (decoded_cmd.u8_data.size() != 1 || !decoded_cmd.i16_data.empty() ||
        !decoded_cmd.i32_data.empty())
      {
        return false;
      }
      frame.data[0] = static_cast<uint8_t>(static_cast<uint8_t>(MaintenanceCommands::MAINTENANCE_CMD) + node_id);
      frame.data[1] = decoded_cmd.u8_data[0];
      return true;
    case MaintenanceCommands::DC_SPECS_CMD:
      if (decoded_cmd.u8_data.size() != 1 || !decoded_cmd.i16_data.empty() ||
        !decoded_cmd.i32_data.empty())
      {
        return false;
      }
      frame.data[0] = static_cast<uint8_t>(static_cast<uint8_t>(MaintenanceCommands::DC_SPECS_CMD) + node_id);
      frame.data[1] = static_cast<uint8_t>(ValidateRequest::VALID);
      return true;
    default:
      return false;
  }
}

// =============================================================================
// Logger
// =============================================================================

void DCHardwareInterface::logger_function()
{
  if (num_joints == 0) return;

  std::string log_msg = "\033[2J\033[H \nDC Motor Logger";
  std::ostringstream oss;
  std::string control_mode;
  std::string joint_type_str;
  std::string status;

  oss << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface
      << " | Command CAN ID: 0x" << std::hex << std::uppercase << can_command_id
      << " | Response CAN ID: 0x" << std::hex << std::uppercase << can_response_id
      << " | HWI Update Rate: " << std::dec << update_rate
      << " | Logger Update Rate: " << logger_rate << "\n"
      << "Elapsed Time since first update: " << elapsed_time << "\n"
      << "\n--- Joint Specific ---";

  for (int i = 0; i < num_joints; i++) {
    const auto & joint = DCJoints_[static_cast<size_t>(i)];
    switch (joint.control_level) {
      case integration_level_t::POSITION:  control_mode = "POSITION"; break;
      case integration_level_t::VELOCITY:  control_mode = "VELOCITY"; break;
      default:                             control_mode = "UNDEFINED"; break;
    }

    switch (joint.joint_type) {
      case joint_type_t::REVOLUTE:  joint_type_str = "REVOLUTE"; break;
      case joint_type_t::PRISMATIC: joint_type_str = "PRISMATIC"; break;
    }

    switch (joint.device_status) {
      case static_cast<int>(MotorStatus::UNDEFINED):               status = "Undefined"; break;
      case static_cast<int>(MotorStatus::IDLE):                    status = "Idle"; break;
      case static_cast<int>(MotorStatus::STARTUP_SEQUENCE):        status = "Startup Sequence"; break;
      case static_cast<int>(MotorStatus::ERROR_INVALID_REQUEST):   status = "Error (Invalid Request)"; break;
      case static_cast<int>(MotorStatus::ERROR_MOTOR_DISARMED):    status = "Error (Motor Disarmed)"; break;
      case static_cast<int>(MotorStatus::ERROR_MOTOR_FAILED):      status = "Error (Motor Failed)"; break;
      case static_cast<int>(MotorStatus::ERROR_CONTROLLER_FAILED): status = "Error (Controller Failed)"; break;
      case static_cast<int>(MotorStatus::ERROR_ESTOP_REQUESTED):   status = "Error (ESTOP Requested)"; break;
      case static_cast<int>(MotorStatus::ERROR_UNKNOWN_POSITION):  status = "Error (Unknown Position)"; break;
      case static_cast<int>(MotorStatus::POSITION_CONTROL):        status = "Position Control"; break;
      case static_cast<int>(MotorStatus::VELOCITY_CONTROL):        status = "Velocity Control"; break;
      case static_cast<int>(MotorStatus::MOTOR_STOPPED):           status = "Motor Stopped"; break;
      default: status = "UNKNOWN (" + std::to_string(joint.device_status) + ")"; break;
    }

    oss << "\nJOINT: " << joint.name << "\n"
        << "Parameters: Node ID: 0x" << std::hex << std::uppercase << static_cast<int>(joint.node_id)
        << " | Gear Ratio: " << std::dec << joint.gear_ratio
        << " | Inverted: " << (joint.inverted ? "true" : "false")
        << " | Device Status: " << status
        << " | Control Mode: " << control_mode
        << " | Joint Type: " << joint_type_str << "\n"
        << "-- Commands --\n"
        << "Joint Command Position: " << joint.joint_command_position
        << " | Joint Command Velocity: " << joint.joint_command_velocity << "\n"
        << "-- State --\n"
        << "Joint Position: " << joint.joint_state_position
        << " | Joint Velocity: " << joint.joint_state_velocity << "\n"
        << "-- Telemetry --\n"
        << "Temperature: " << joint.motor_temperature << " C"
        << " | Torque Current: " << joint.motor_torque_current << " A\n";
  }

  log_msg += oss.str();
  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "%s", log_msg.c_str());
}

// =============================================================================
// Initialization
// =============================================================================

hardware_interface::CallbackReturn DCHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  DCJoints_.clear();
  for (const auto & joint : info_.joints) {
    std::vector<std::string> state_if_names;
    for (const auto & si : joint.state_interfaces) {
      state_if_names.push_back(si.name);
    }

    std::vector<std::string> command_if_names;
    for (const auto & ci : joint.command_interfaces) {
      command_if_names.push_back(ci.name);
    }

    std::unordered_map<std::string, std::string> params_map;
    for (const auto & p : joint.parameters) {
      params_map.emplace(p.first, p.second);
    }

    const uint8_t node_id = static_cast<uint8_t>(
      std::clamp(std::stoi(joint.parameters.at("node_id"), nullptr, 0), 0, 15));
    const int gear_ratio = joint.parameters.count("gear_ratio") ?
      std::abs(std::stoi(joint.parameters.at("gear_ratio"))) : 1;
    const bool inverted = joint.parameters.count("inverted") &&
      joint.parameters.at("inverted") == "true";

    joint_type_t joint_type = joint_type_t::REVOLUTE;
    double joint_max_disp = std::numeric_limits<double>::quiet_NaN();
    double joint_distance_per_rev = 0.0;
    const std::string jtype = joint.parameters.count("joint_type") ?
      joint.parameters.at("joint_type") : "revolute";
    if (jtype == "revolute") {
      joint_type = joint_type_t::REVOLUTE;
    } else if (
      jtype == "prismatic" &&
      joint.parameters.count("max_disp") &&
      joint.parameters.count("distance_per_rev"))
    {
      joint_type = joint_type_t::PRISMATIC;
      joint_max_disp = std::abs(std::stod(joint.parameters.at("max_disp")));
      joint_distance_per_rev = std::stod(joint.parameters.at("distance_per_rev"));
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("DCHardwareInterface"),
        "Invalid joint_type for joint %s. Must be 'revolute' or 'prismatic' (with 'max_disp' and 'distance_per_rev').",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const integration_level_t initial_mode =
      jtype == "continuous" ? integration_level_t::VELOCITY : integration_level_t::POSITION;

    DCJoints_.push_back(DCJoint{
      joint.name,
      node_id,
      gear_ratio,
      inverted,
      joint_max_disp,
      joint_distance_per_rev,
      initial_mode,
      joint_type,
      std::numeric_limits<double>::quiet_NaN(),
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      std::numeric_limits<double>::quiet_NaN(),
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      {},
      0.0,
      0.0,
      0.0,
      0.0,
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      static_cast<int>(MotorStatus::UNDEFINED),
      state_if_names,
      command_if_names,
      params_map
    });
  }

  // -- Hardware-level parameters --
  num_joints = static_cast<int>(info_.joints.size());
  update_rate = std::stoi(info_.hardware_parameters.at("update_rate"));
  logger_rate = std::stoi(info_.hardware_parameters.at("logger_rate"));
  logger_state = std::stoi(info_.hardware_parameters.at("logger_state"));
  can_interface = info_.hardware_parameters.at("can_interface");
  can_command_id = std::stoi(info_.hardware_parameters.at("can_id"), nullptr, 0);
  can_response_id = can_command_id + 0x01;

  elapsed_update_time = 0.0;
  elapsed_time = 0.0;
  elapsed_logger_time = 0.0;
  write_count = 0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// Lifecycle
// =============================================================================

hardware_interface::CallbackReturn DCHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return this->on_cleanup(previous_state);
}

std::vector<hardware_interface::StateInterface>
DCHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < DCJoints_.size(); i++) {
    auto & joint = DCJoints_[i];
    for (const auto & iface : joint.state_interface_names) {
      double * value = nullptr;
      if (iface == hardware_interface::HW_IF_POSITION) {
        value = &joint.joint_state_position;
      } else if (iface == hardware_interface::HW_IF_VELOCITY) {
        value = &joint.joint_state_velocity;
      } else if (iface == "status") {
        value = &joint.motor_status;
      } else if (iface == "motor_temperature") {
        value = &joint.motor_temperature;
      } else if (iface == "torque_current") {
        value = &joint.motor_torque_current;
      }

      if (value == nullptr) {
        continue;
      }
      state_interfaces.emplace_back(joint.name, iface, value);
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DCHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < DCJoints_.size(); i++) {
    auto & joint = DCJoints_[i];
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
      }

      if (value == nullptr) {
        continue;
      }
      command_interfaces.emplace_back(joint.name, iface, value);
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DCHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "Configuring DC Motor HWI...");

  if (!canBus.open(can_interface,
        std::bind(&DCHardwareInterface::on_can_message, this, std::placeholders::_1)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("DCHardwareInterface"),
      "Failed to open CAN interface: %s", can_interface.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "CAN interface opened successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DCHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "Cleaning up... please wait...");

  for (const auto & joint : DCJoints_) {
    CANLib::CanFrame frame;
    frame.id = can_command_id;
    if (format_maintenance_command(
        frame, joint.node_id,
        DecodedCommand{
          static_cast<uint8_t>(MaintenanceCommands::MAINTENANCE_CMD),
          {static_cast<uint8_t>(MaintenanceCommandOptions::MOTOR_SHUTDOWN_CMD)},
          {},
          {}
        }))
    {
      canBus.send(frame);
    }
  }

  canBus.close();
  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "Cleanup complete.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DCHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "Activating... please wait...");

  // Initialize command positions to current state
  for (auto & joint : DCJoints_) {
    joint.joint_command_position = joint.joint_state_position;
    joint.joint_command_velocity = 0.0;
  }

  for (size_t i = 0; i < DCJoints_.size(); ++i) {
    RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"),
      "Joint %zu command position initialized to: %f", i, DCJoints_[i].joint_command_position);
  }

  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DCHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "Deactivating... please wait...");

  for (const auto & joint : DCJoints_) {
    CANLib::CanFrame frame;
    frame.id = can_command_id;
    if (format_maintenance_command(
        frame, joint.node_id,
        DecodedCommand{
          static_cast<uint8_t>(MaintenanceCommands::MAINTENANCE_CMD),
          {static_cast<uint8_t>(MaintenanceCommandOptions::MOTOR_STOP_CMD)},
          {},
          {}
        }))
    {
      canBus.send(frame);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// CAN Receive Callback
// =============================================================================

void DCHardwareInterface::on_can_message(const CANLib::CanFrame& frame)
{
  can_rx_frame_ = frame;

  if (frame.id != can_response_id) {
    return;
  }

  const uint8_t command_nibble = static_cast<uint8_t>((frame.data[0] >> 4) & 0x0F);
  const uint8_t device_id_nibble = static_cast<uint8_t>(frame.data[0] & 0x0F);

  for (size_t i = 0; i < DCJoints_.size(); i++) {
    auto & joint = DCJoints_[i];
    if (joint.node_id != device_id_nibble) {
      continue;
    }

    if (command_nibble == (static_cast<uint8_t>(StatusCommands::MOTOR_STATE) >> 4) ||
        command_nibble == (static_cast<uint8_t>(ControlCommands::ABSOLUTE_POS_CONTROL_CMD) >> 4) ||
        command_nibble == (static_cast<uint8_t>(ControlCommands::VELOCITY_CONTROL_CMD) >> 4)) {
      const double raw_motor_position = static_cast<double>(
        static_cast<int16_t>((frame.data[2] << 8) | frame.data[1]));
      const double raw_motor_velocity = static_cast<double>(
        static_cast<int16_t>((frame.data[4] << 8) | frame.data[3]));
      const double dir = joint.inverted ? -1.0 : 1.0;

      if (joint.joint_type == joint_type_t::REVOLUTE) {
        joint.motor_position = dir * calculate_joint_position_from_motor_position(
          raw_motor_position, joint.gear_ratio);
        joint.motor_velocity = dir * calculate_joint_angular_velocity_from_motor_velocity(
          raw_motor_velocity, joint.gear_ratio);
      } else {
        joint.motor_position = dir * calculate_joint_displacement_from_motor_position(
          raw_motor_position, joint.gear_ratio, joint.distance_per_rev);
        joint.motor_velocity = dir * calculate_joint_linear_velocity_from_motor_velocity(
          raw_motor_velocity, joint.gear_ratio, joint.distance_per_rev);
      }
      return;
    }

    if (command_nibble == (static_cast<uint8_t>(StatusCommands::MOTOR_STATUS) >> 4)) {
      joint.device_status = frame.data[1];
      joint.motor_status = static_cast<double>(frame.data[1]);
      return;
    }

    if (command_nibble == (static_cast<uint8_t>(MaintenanceCommands::MAINTENANCE_CMD) >> 4) && frame.data[0] == 1) {
      RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "Successfully sent maintenance command");
      return;
    }

    if (command_nibble == (static_cast<uint8_t>(MaintenanceCommands::DC_SPECS_CMD) >> 4)) {
      const uint8_t command_id = frame.data[0];
      const uint8_t motor_type = frame.data[1];

      const uint16_t max_motor_position =
        static_cast<uint16_t>(frame.data[2]) |
        (static_cast<uint16_t>(frame.data[3]) << 8);

      const uint16_t max_motor_velocity =
        static_cast<uint16_t>(frame.data[4]) |
        (static_cast<uint16_t>(frame.data[5]) << 8);

      RCLCPP_INFO(
        rclcpp::get_logger("DCHardwareInterface"),
        "DC Config Reply | "
        "command_id: 0x%02X (%u) | "
        "motor_type: %u | "
        "max_motor_position: %u | "
        "max_motor_velocity: %u",
        command_id,
        command_id,
        motor_type,
        max_motor_position,
        max_motor_velocity
      );
      return;
    }
  }
}

// =============================================================================
// Read (update state from CAN data)
// =============================================================================

hardware_interface::return_type DCHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (auto & joint : DCJoints_) {
    joint.joint_state_velocity = joint.motor_velocity;
    joint.joint_state_position = joint.motor_position;

    // Check device status — allow Undefined(0), Idle(1), Position Control(9), Velocity Control(10)
    const int s = joint.device_status;
    if (s != 0 && s != 1 && s != 9 && s != 10) {
      RCLCPP_ERROR(rclcpp::get_logger("DCHardwareInterface"),
        "Joint %s in error state: %d", joint.name.c_str(), s);
      return hardware_interface::return_type::ERROR;
    }

    // TODO: Telemetry — populate motor_temperature and motor_torque_current via CAN
  }

  return hardware_interface::return_type::OK;
}

// =============================================================================
// Write (send CAN commands)
// =============================================================================

hardware_interface::return_type DCHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  elapsed_time += period.seconds();

  elapsed_logger_time += period.seconds();
  if (logger_rate > 0 && elapsed_logger_time > (1.0 / static_cast<double>(logger_rate))) {
    elapsed_logger_time = 0.0;
    if (logger_state == 1) {
      logger_function();
    }
  }

  for (size_t i = 0; i < DCJoints_.size(); ++i) {
    auto & joint = DCJoints_[i];
    const double curr_status_req = joint.motor_status_req;
    if (curr_status_req < 0.0 && joint.prev_status_req >= 0.0) {
      for (auto status_cmd : kStatusCommands) {
        CANLib::CanFrame frame;
        frame.id = can_command_id;
        if (format_status_command(frame, static_cast<uint8_t>(status_cmd), joint.node_id)) {
          canBus.send(frame);
        }
      }
    } else if (curr_status_req > 0.0) {
      joint.elapsed_status_request_time += period.seconds();
      if (joint.elapsed_status_request_time > (1.0 / curr_status_req)) {
        joint.elapsed_status_request_time = 0.0;
        for (auto status_cmd : kStatusCommands) {
          CANLib::CanFrame frame;
          frame.id = can_command_id;
          if (format_status_command(frame, static_cast<uint8_t>(status_cmd), joint.node_id)) {
            canBus.send(frame);
          }
        }
      }
    }
    joint.prev_status_req = curr_status_req;
  }

  for (size_t i = 0; i < DCJoints_.size(); ++i) {
    auto & joint = DCJoints_[i];
    auto doubles_to_payload = [](double high, double low) -> int64_t {
      return static_cast<int64_t>(
        (static_cast<uint64_t>(high) << 32) | static_cast<uint64_t>(low));
    };

    joint.maintenance_frame = static_cast<double>(doubles_to_payload(
      joint.maintenance_frame_high, joint.maintenance_frame_low));
    const auto decoded_maintenance_cmd = unpack_command_full(
      static_cast<int32_t>(joint.maintenance_data_count),
      static_cast<int64_t>(joint.maintenance_frame));
    joint.decoded_maintenance_frame.clear();
    joint.decoded_maintenance_frame.push_back(decoded_maintenance_cmd.command_id);
    joint.decoded_maintenance_frame.insert(
      joint.decoded_maintenance_frame.end(),
      decoded_maintenance_cmd.u8_data.begin(),
      decoded_maintenance_cmd.u8_data.end());

    CANLib::CanFrame frame;
    frame.id = can_command_id;
    if (!format_maintenance_command(frame, joint.node_id, decoded_maintenance_cmd)) {
      joint.prev_maintenance_req = joint.motor_maintenance_req;
      continue;
    }

    const double curr_maintenance_req = joint.motor_maintenance_req;
    if (curr_maintenance_req < 0.0 && joint.prev_maintenance_req >= 0.0) {
      canBus.send(frame);
    } else if (curr_maintenance_req > 0.0) {
      joint.elapsed_maintenance_request_time += period.seconds();
      if (joint.elapsed_maintenance_request_time > (1.0 / curr_maintenance_req)) {
        joint.elapsed_maintenance_request_time = 0.0;
        canBus.send(frame);
      }
    }
    joint.prev_maintenance_req = curr_maintenance_req;
  }

  elapsed_update_time += period.seconds();
  if (update_rate > 0 && elapsed_update_time > (1.0 / static_cast<double>(update_rate))) {
    elapsed_update_time = 0.0;
    if (!DCJoints_.empty()) {
      const size_t joint_index = static_cast<size_t>(write_count % num_joints);
      ++write_count;
      CANLib::CanFrame frame;
      frame.id = can_command_id;
      format_control_command(frame, joint_index);
      canBus.send(frame);
    }
  }

  return hardware_interface::return_type::OK;
}

// =============================================================================
// Command Mode Switch
// =============================================================================

hardware_interface::return_type DCHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string>& start_interfaces,
  const std::vector<std::string>& stop_interfaces)
{
  std::ostringstream ss;
  ss << "perform_command_mode_switch called. start: [";
  for (auto &s : start_interfaces) ss << s << ",";
  ss << "] stop: [";
  for (auto &s : stop_interfaces) ss << s << ",";
  ss << "]";
  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "%s", ss.str().c_str());

  std::vector<integration_level_t> requested_modes(num_joints, integration_level_t::UNDEFINED);

  // Process stop interfaces
  for (const auto &ifname : stop_interfaces) {
    for (int i = 0; i < num_joints; ++i) {
      if (ifname.find(DCJoints_[static_cast<size_t>(i)].name) != std::string::npos) {
        requested_modes[i] = integration_level_t::UNDEFINED;
      }
    }
  }

  // Process start interfaces
  for (const auto &ifname : start_interfaces) {
    for (int i = 0; i < num_joints; ++i) {
      const auto & joint = DCJoints_[static_cast<size_t>(i)];
      const std::string pos_if = joint.name + "/" + std::string(hardware_interface::HW_IF_POSITION);
      const std::string vel_if = joint.name + "/" + std::string(hardware_interface::HW_IF_VELOCITY);
      if (ifname == pos_if) {
        requested_modes[i] = integration_level_t::POSITION;
      } else if (ifname == vel_if) {
        requested_modes[i] = integration_level_t::VELOCITY;
      }
    }
  }

  // Apply modes
  for (int i = 0; i < num_joints; ++i) {
    auto & joint = DCJoints_[static_cast<size_t>(i)];
    if (requested_modes[i] == integration_level_t::UNDEFINED) {
      bool was_stopped = false;
      for (const auto &ifname : stop_interfaces) {
        if (ifname.find(joint.name) != std::string::npos) {
          was_stopped = true;
          break;
        }
      }
      if (was_stopped) {
        joint.control_level = integration_level_t::UNDEFINED;
        joint.joint_command_velocity = 0.0;
        joint.joint_command_position = joint.joint_state_position;
        RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"),
          "Joint %s: stopped -> UNDEFINED", joint.name.c_str());
      }
    } else {
      joint.control_level = requested_modes[i];
      if (requested_modes[i] == integration_level_t::VELOCITY) {
        joint.joint_command_velocity = 0.0;
        RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"),
          "Joint %s: switched to VELOCITY", joint.name.c_str());
      } else if (requested_modes[i] == integration_level_t::POSITION) {
        joint.joint_command_position = joint.joint_state_position;
        RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"),
          "Joint %s: switched to POSITION (initialized to %f)",
          joint.name.c_str(), joint.joint_command_position);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace dc_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dc_ros2_control::DCHardwareInterface, hardware_interface::SystemInterface)

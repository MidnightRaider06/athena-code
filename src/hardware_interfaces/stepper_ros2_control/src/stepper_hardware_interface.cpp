#include "stepper_ros2_control/stepper_hardware_interface.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace stepper_ros2_control
{

double STEPPERHardwareInterface::calculate_joint_position_from_motor_position(
  double motor_position, int gear_ratio)
{
  return (motor_position * (M_PI / 180.0)) / gear_ratio;
}

double STEPPERHardwareInterface::calculate_joint_velocity_from_motor_velocity(
  double motor_velocity, int gear_ratio)
{
  return (motor_velocity * (M_PI / 180.0)) / gear_ratio;
}

int16_t STEPPERHardwareInterface::calculate_motor_position_from_desired_joint_position(
  double joint_position, int gear_ratio)
{
  return static_cast<int16_t>(std::round((joint_position * (180.0 / M_PI)) * gear_ratio));
}

int16_t STEPPERHardwareInterface::calculate_motor_velocity_from_desired_joint_velocity(
  double joint_velocity, int gear_ratio)
{
  return static_cast<int16_t>(std::round((joint_velocity * (180.0 / M_PI)) * gear_ratio));
}

void STEPPERHardwareInterface::logger_function()
{
  if (num_joints == 0) {
    return;
  }

  std::ostringstream oss;
  oss << "\033[2J\033[H \nSTEPPER Logger"
      << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface
      << " | Command CAN ID: 0x" << std::hex << std::uppercase << can_command_id
      << " | Response CAN ID: 0x" << can_response_id << std::dec
      << " | HWI Update Rate: " << update_rate
      << " | Logger Update Rate: " << logger_rate << "\n"
      << "Elapsed Time since first update: " << elapsed_time << "\n"
      << "\n--- Joint Specific ---";

  for (const auto & joint : STEPPERJoints_) {
    std::string control_mode = "UNDEFINED";
    if (joint.control_level == integration_level_t::POSITION) {
      control_mode = "POSITION";
    } else if (joint.control_level == integration_level_t::VELOCITY) {
      control_mode = "VELOCITY";
    }

    oss << "\nJOINT: " << joint.name << "\n"
        << "Parameters: Node ID: 0x" << std::hex << std::uppercase
        << static_cast<int>(joint.node_id) << std::dec
        << " | Gear Ratio: " << joint.gear_ratio
        << " | Orientation: " << joint.orientation
        << " | Control Mode: " << control_mode << "\n"
        << "-- Commands --\n"
        << "Motor Position: " << joint.motor_position
        << " | Joint Command Position: " << joint.joint_command_position << "\n"
        << "Motor Velocity: " << joint.motor_velocity
        << " | Joint Command Velocity: " << joint.joint_command_velocity << "\n"
        << "Status Request: " << joint.motor_status_req
        << " | Maintenance Request: " << joint.motor_maintenance_req << "\n"
        << "-- State --\n"
        << "Joint Position: " << joint.joint_state_position
        << " | Joint Velocity: " << joint.joint_state_velocity
        << " | Status: " << joint.motor_status << "\n";
  }

  RCLCPP_INFO(rclcpp::get_logger("STEPPERHardwareInterface"), "%s", oss.str().c_str());
}

void STEPPERHardwareInterface::format_control_command(CANLib::CanFrame & frame, StepperJoint & joint)
{
  std::fill(std::begin(frame.data), std::end(frame.data), 0x00);

  if (
    joint.control_level == integration_level_t::POSITION &&
    std::isfinite(joint.joint_command_position) &&
    joint.joint_command_position != joint.prev_joint_command_position)
  {
    joint.prev_joint_command_position = joint.joint_command_position;
    const int16_t joint_position = static_cast<int16_t>(
      joint.orientation *
      calculate_motor_position_from_desired_joint_position(
        joint.joint_command_position, joint.gear_ratio));

    frame.dlc = 3;
    frame.data[0] = static_cast<uint8_t>(
      static_cast<uint8_t>(ControlCommands::RELATIVE_POS_CONTROL_CMD) + joint.node_id);
    frame.data[1] = static_cast<uint8_t>(joint_position & 0xFF);
    frame.data[2] = static_cast<uint8_t>((joint_position >> 8) & 0xFF);
    return;
  }
  else if (
    joint.control_level == integration_level_t::VELOCITY &&
    std::isfinite(joint.joint_command_velocity) &&
    joint.joint_command_velocity != joint.prev_joint_command_velocity)
  {
    joint.prev_joint_command_velocity = joint.joint_command_velocity;
    int16_t joint_velocity = static_cast<int16_t>(
      joint.orientation *
      calculate_motor_velocity_from_desired_joint_velocity(
        joint.joint_command_velocity, joint.gear_ratio));

    // TEMPORARY: AUTOMATICALLY SET VALUES TO 900/0/-900 BASED ON SIGN OF VELOCITY COMMAND
    if (joint_velocity > 0) {
      joint_velocity = 900;
    } else if (joint_velocity < 0) {
      joint_velocity = -900;
    }
    else {
      joint_velocity = 0;
    }
    // TEMPORARY END

    frame.dlc = 3;
    frame.data[0] = static_cast<uint8_t>(
      static_cast<uint8_t>(ControlCommands::VELOCITY_CONTROL_CMD) + joint.node_id);
    frame.data[1] = static_cast<uint8_t>(joint_velocity & 0xFF);
    frame.data[2] = static_cast<uint8_t>((joint_velocity >> 8) & 0xFF);
    return;
  }
  else {
    frame.dlc = 2;
    frame.data[0] = static_cast<uint8_t>(
      static_cast<uint8_t>(StatusCommands::MOTOR_STATE) + joint.node_id);
    frame.data[1] = static_cast<uint8_t>(ValidateRequest::VALID);
  }

}

bool STEPPERHardwareInterface::format_status_command(
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

bool STEPPERHardwareInterface::format_maintenance_command(
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
    case MaintenanceCommands::STEPPER_SPECS_CMD:
      if (decoded_cmd.u8_data.size() != 1 || !decoded_cmd.i16_data.empty() ||
        !decoded_cmd.i32_data.empty())
      {
        return false;
      }
      frame.data[0] = static_cast<uint8_t>(static_cast<uint8_t>(MaintenanceCommands::STEPPER_SPECS_CMD) + node_id);
      frame.data[1] = static_cast<uint8_t>(ValidateRequest::VALID);
      return true;
    default:
      return false;
  }
}

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  STEPPERJoints_.clear();
  update_rate = std::stoi(info_.hardware_parameters.at("update_rate"));
  logger_rate = std::stoi(info_.hardware_parameters.at("logger_rate"));
  logger_state = std::stoi(info_.hardware_parameters.at("logger_state"));
  can_interface = info_.hardware_parameters.at("can_interface");
  can_command_id = std::stoi(info_.hardware_parameters.at("can_id"), nullptr, 0);
  can_response_id = static_cast<uint32_t>(can_command_id + 0x01);

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

    const int orientation = joint.parameters.count("joint_orientation") &&
      std::stoi(joint.parameters.at("joint_orientation")) == -1 ? -1 : 1;
    const integration_level_t initial_mode =
      joint.parameters.count("joint_type") && joint.parameters.at("joint_type") == "continuous" ?
      integration_level_t::VELOCITY : integration_level_t::POSITION;

    STEPPERJoints_.push_back(StepperJoint{
      joint.name,
      static_cast<uint8_t>(std::clamp(std::stoi(joint.parameters.at("node_id"), nullptr, 0), 0, 15)),
      std::abs(std::stoi(joint.parameters.at("gear_ratio"))),
      orientation,
      initial_mode,
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
      state_if_names,
      command_if_names,
      params_map
    });
  }

  num_joints = static_cast<int>(STEPPERJoints_.size());
  elapsed_update_time = 0.0;
  elapsed_time = 0.0;
  elapsed_logger_time = 0.0;
  write_count = 0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

std::vector<hardware_interface::StateInterface> STEPPERHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & joint : STEPPERJoints_) {
    for (const auto & iface : joint.state_interface_names) {
      double * value = nullptr;
      if (iface == hardware_interface::HW_IF_POSITION) {
        value = &joint.joint_state_position;
      } else if (iface == hardware_interface::HW_IF_VELOCITY) {
        value = &joint.joint_state_velocity;
      } else if (iface == "status") {
        value = &joint.motor_status;
      }

      if (value == nullptr) {
        continue;
      }
      state_interfaces.emplace_back(joint.name, iface, value);
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> STEPPERHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto & joint : STEPPERJoints_) {
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

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!canBus.open(
      can_interface,
      std::bind(&STEPPERHardwareInterface::on_can_message, this, std::placeholders::_1)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("STEPPERHardwareInterface"), "Failed to open CAN interface");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

void STEPPERHardwareInterface::on_can_message(const CANLib::CanFrame & frame)
{
  can_rx_frame_ = frame;
  if (frame.id != can_response_id) {
    return;
  }

  const uint8_t command_nibble = static_cast<uint8_t>((frame.data[0] >> 4) & 0x0F);
  const uint8_t device_id_nibble = static_cast<uint8_t>(frame.data[0] & 0x0F);

  for (auto & joint : STEPPERJoints_) {
    if (joint.node_id != device_id_nibble) {
      continue;
    }

    if (command_nibble == (static_cast<uint8_t>(StatusCommands::MOTOR_STATE) >> 4) ||
        command_nibble == (static_cast<uint8_t>(ControlCommands::RELATIVE_POS_CONTROL_CMD) >> 4) ||
        command_nibble == (static_cast<uint8_t>(ControlCommands::VELOCITY_CONTROL_CMD) >> 4)) {
      const double raw_position = static_cast<double>(
        static_cast<int16_t>((frame.data[2] << 8) | frame.data[1]));
      const double raw_velocity = static_cast<double>(
        static_cast<int16_t>((frame.data[4] << 8) | frame.data[3]));

      joint.motor_position = joint.orientation *
        calculate_joint_position_from_motor_position(raw_position, joint.gear_ratio);
      joint.motor_velocity = joint.orientation *
        calculate_joint_velocity_from_motor_velocity(raw_velocity, joint.gear_ratio);
      return;
    }

    if (command_nibble == (static_cast<uint8_t>(StatusCommands::MOTOR_STATUS) >> 4)) {
      joint.motor_status = static_cast<double>(frame.data[1]);
      return;
    }

    if (command_nibble == (static_cast<uint8_t>(MaintenanceCommands::MAINTENANCE_CMD) >> 4) && frame.data[0] == 1) {
      RCLCPP_INFO(rclcpp::get_logger("STEPPERHardwareInterface"), "Successfully sent maintenance command");
      return;
    }

    if (command_nibble == (static_cast<uint8_t>(MaintenanceCommands::STEPPER_SPECS_CMD) >> 4)) {
      const uint8_t command_id = frame.data[0];
      const uint8_t stepper_type = frame.data[1];

      const uint16_t max_stepper_position =
        static_cast<uint16_t>(frame.data[2]) |
        (static_cast<uint16_t>(frame.data[3]) << 8);

      const uint16_t max_stepper_velocity =
        static_cast<uint16_t>(frame.data[4]) |
        (static_cast<uint16_t>(frame.data[5]) << 8);

      RCLCPP_INFO(
        rclcpp::get_logger("STEPPERHardwareInterface"),
        "Stepper Config Reply | "
        "command_id: 0x%02X (%u) | "
        "stepper_type: %u | "
        "max_stepper_position: %u | "
        "max_stepper_velocity: %u",
        command_id,
        command_id,
        stepper_type,
        max_stepper_position,
        max_stepper_velocity
      );
      return;
    }
  }
}

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  for (const auto & joint : STEPPERJoints_) {
    CANLib::CanFrame frame;
    frame.id = can_command_id;
    if (format_maintenance_command(
        frame, joint.node_id,
        DecodedCommand{
          static_cast<uint8_t>(MaintenanceCommands::MAINTENANCE_CMD),
          {static_cast<uint8_t>(MaintenanceCommandOptions::MOTOR_SHUTDOWN_CMD)},
          {},
          {}}))
    {
      canBus.send(frame);
    }
  }
  canBus.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  for (auto & joint : STEPPERJoints_) {
    joint.joint_command_position = joint.joint_state_position;
    joint.joint_command_velocity = 0.0;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (const auto & joint : STEPPERJoints_) {
    CANLib::CanFrame frame;
    frame.id = can_command_id;
    if (format_maintenance_command(
        frame, joint.node_id,
        DecodedCommand{
          static_cast<uint8_t>(MaintenanceCommands::MAINTENANCE_CMD),
          {static_cast<uint8_t>(MaintenanceCommandOptions::MOTOR_STOP_CMD)},
          {},
          {}}))
    {
      canBus.send(frame);
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type STEPPERHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto & joint : STEPPERJoints_) {
    joint.joint_state_position = joint.motor_position;
    joint.joint_state_velocity = joint.motor_velocity;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type STEPPERHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  elapsed_time += period.seconds();
  elapsed_logger_time += period.seconds();
  if (logger_rate > 0 && elapsed_logger_time > (1.0 / static_cast<double>(logger_rate))) {
    elapsed_logger_time = 0.0;
    if (logger_state == 1) {
      logger_function();
    }
  }

  for (auto & joint : STEPPERJoints_) {
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

  for (auto & joint : STEPPERJoints_) {
    auto doubles_to_payload = [](double high, double low) -> int64_t {
      return static_cast<int64_t>(
        (static_cast<uint64_t>(high) << 32) | static_cast<uint64_t>(low));
    };

    joint.maintenance_frame = static_cast<double>(doubles_to_payload(
      joint.maintenance_frame_high, joint.maintenance_frame_low));
    const auto decoded_maintenance_cmd = unpack_command_full(
      static_cast<int32_t>(joint.maintenance_data_count),
      static_cast<int64_t>(joint.maintenance_frame));
    pack_decoded_maintenance_frame(joint, decoded_maintenance_cmd);

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
    if (!STEPPERJoints_.empty()) {
      auto & joint = STEPPERJoints_[static_cast<size_t>(write_count % num_joints)];
      ++write_count;
      CANLib::CanFrame frame;
      frame.id = can_command_id;
      format_control_command(frame, joint);
      canBus.send(frame);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type STEPPERHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  std::vector<integration_level_t> requested_modes(
    static_cast<size_t>(num_joints), integration_level_t::UNDEFINED);

  for (const auto & ifname : stop_interfaces) {
    for (size_t i = 0; i < STEPPERJoints_.size(); ++i) {
      const auto & joint = STEPPERJoints_[i];
      const std::string pos_if = joint.name + "/" + hardware_interface::HW_IF_POSITION;
      const std::string vel_if = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
      if (ifname == pos_if || ifname == vel_if || ifname.find(joint.name) != std::string::npos) {
        requested_modes[i] = integration_level_t::UNDEFINED;
      }
    }
  }

  for (const auto & ifname : start_interfaces) {
    for (size_t i = 0; i < STEPPERJoints_.size(); ++i) {
      const auto & joint = STEPPERJoints_[i];
      const std::string pos_if = joint.name + "/" + hardware_interface::HW_IF_POSITION;
      const std::string vel_if = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
      if (ifname == pos_if) {
        requested_modes[i] = integration_level_t::POSITION;
      } else if (ifname == vel_if) {
        requested_modes[i] = integration_level_t::VELOCITY;
      }
    }
  }

  for (size_t i = 0; i < STEPPERJoints_.size(); ++i) {
    auto & joint = STEPPERJoints_[i];
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
    } else {
      joint.joint_command_position = joint.joint_state_position;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace stepper_ros2_control

PLUGINLIB_EXPORT_CLASS(
  stepper_ros2_control::STEPPERHardwareInterface, hardware_interface::SystemInterface)
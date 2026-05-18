#include "rotary_encoder_ros2_control/rotary_encoder_hardware_interface.hpp"

#include <cmath>
#include <limits>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rotary_encoder_ros2_control
{

double RotaryEncoderHardwareInterface::position_from_raw(int16_t raw_position)
{
  return static_cast<double>(raw_position) * 0.1 * (M_PI / 180.0);
}

double RotaryEncoderHardwareInterface::velocity_from_raw(int16_t raw_velocity)
{
  return static_cast<double>(raw_velocity) * 0.01 * (M_PI / 180.0);
}

void RotaryEncoderHardwareInterface::send_state_request(const RotaryEncoderJoint & joint)
{
  CANLib::CanFrame frame;
  frame.id = can_write_id_;
  frame.dlc = 8;
  frame.data.fill(0x00);
  frame.data[0] = static_cast<uint8_t>(
    ROTARY_ENCODER_STATE_CMD + static_cast<uint8_t>(joint.node_id & 0xFF));
  frame.data[1] = joint.rotary_encoder_id;
  canBus.send(frame);
}

void RotaryEncoderHardwareInterface::logger_function()
{
  if (rotary_encoder_joints_.empty()) {
    return;
  }

  std::ostringstream oss;
  oss << "\033[2J\033[H \nRotary Encoder Logger"
      << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface_
      << " | Write CAN ID: 0x" << std::hex << std::uppercase << can_write_id_
      << " | Read CAN ID: 0x" << can_read_id_ << std::dec
      << " | HWI Update Rate: " << update_rate_
      << " | Logger Update Rate: " << logger_rate_ << "\n"
      << "Elapsed Time since first update: " << elapsed_time_
      << "\n--- Joint Specific ---";

  for (const auto & joint : rotary_encoder_joints_) {
    oss << "\nJOINT: " << joint.name
        << " | Node ID: 0x" << std::hex << std::uppercase << joint.node_id
        << " | Rotary Encoder ID: 0x" << static_cast<int>(joint.rotary_encoder_id)
        << std::dec
        << "\n-- Commands --\n"
        << "State Request: " << joint.state_request
        << "\n-- State --\n"
        << "Position: " << joint.joint_state_position
        << " | Velocity: " << joint.joint_state_velocity << "\n";
  }

  RCLCPP_INFO(rclcpp::get_logger("RotaryEncoderHardwareInterface"), "%s", oss.str().c_str());
}

hardware_interface::CallbackReturn RotaryEncoderHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  can_interface_ = info_.hardware_parameters.count("can_interface") ?
    info_.hardware_parameters.at("can_interface") : "can0";
  update_rate_ = info_.hardware_parameters.count("update_rate") ?
    std::stoi(info_.hardware_parameters.at("update_rate")) : 10;
  logger_rate_ = info_.hardware_parameters.count("logger_rate") ?
    std::stoi(info_.hardware_parameters.at("logger_rate")) : 0;
  logger_state_ = info_.hardware_parameters.count("logger_state") ?
    std::stoi(info_.hardware_parameters.at("logger_state")) : 0;

  if (!info_.hardware_parameters.count("can_id")) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RotaryEncoderHardwareInterface"),
      "CAN ID parameter 'can_id' is missing in hardware configuration.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    can_write_id_ = static_cast<uint32_t>(
      std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RotaryEncoderHardwareInterface"),
      "Failed to parse 'can_id': %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  can_read_id_ = can_write_id_ + 1;

  rotary_encoder_joints_.clear();
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

    const uint32_t node_id = static_cast<uint32_t>(
      std::stoul(joint.parameters.at("node_id"), nullptr, 0));
    const uint8_t rotary_encoder_id = static_cast<uint8_t>(
      std::stoul(joint.parameters.at("rotary_encoder_id"), nullptr, 0) & 0xFF);

    rotary_encoder_joints_.push_back(RotaryEncoderJoint{
      joint.name,
      node_id,
      rotary_encoder_id,
      std::numeric_limits<double>::quiet_NaN(),
      0.0,
      0.0,
      0.0,
      0.0,
      state_if_names,
      command_if_names,
      params_map});
  }

  elapsed_time_ = 0.0;
  elapsed_logger_time_ = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RotaryEncoderHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!canBus.open(
      can_interface_,
      std::bind(&RotaryEncoderHardwareInterface::on_can_message, this, std::placeholders::_1)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("RotaryEncoderHardwareInterface"), "Failed to open CAN interface");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RotaryEncoderHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & joint : rotary_encoder_joints_) {
    for (const auto & iface : joint.state_interface_names) {
      double * value = nullptr;
      if (iface == hardware_interface::HW_IF_POSITION) {
        value = &joint.joint_state_position;
      } else if (iface == hardware_interface::HW_IF_VELOCITY) {
        value = &joint.joint_state_velocity;
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("RotaryEncoderHardwareInterface"),
          "Unknown state interface '%s' for joint '%s'",
          iface.c_str(), joint.name.c_str());
        continue;
      }
      state_interfaces.emplace_back(joint.name, iface, value);
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RotaryEncoderHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto & joint : rotary_encoder_joints_) {
    for (const auto & iface : joint.command_interface_names) {
      if (iface == "state_request") {
        command_interfaces.emplace_back(joint.name, iface, &joint.state_request);
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("RotaryEncoderHardwareInterface"),
          "Unknown command interface '%s' for joint '%s'",
          iface.c_str(), joint.name.c_str());
      }
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RotaryEncoderHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RotaryEncoderHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RotaryEncoderHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  canBus.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RotaryEncoderHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

void RotaryEncoderHardwareInterface::on_can_message(const CANLib::CanFrame & frame)
{
  can_rx_frame_ = frame;
  if (can_rx_frame_.id != can_read_id_) {
    return;
  }

  for (auto & joint : rotary_encoder_joints_) {
    const uint8_t expected_command = static_cast<uint8_t>(
      ROTARY_ENCODER_STATE_CMD + static_cast<uint8_t>(joint.node_id & 0xFF));
    if (
      can_rx_frame_.data[0] != expected_command ||
      can_rx_frame_.data[1] != joint.rotary_encoder_id)
    {
      continue;
    }

    const auto raw_position = static_cast<int16_t>(
      (static_cast<uint16_t>(can_rx_frame_.data[3]) << 8) |
      static_cast<uint16_t>(can_rx_frame_.data[2]));
    const auto raw_velocity = static_cast<int16_t>(
      (static_cast<uint16_t>(can_rx_frame_.data[5]) << 8) |
      static_cast<uint16_t>(can_rx_frame_.data[4]));

    joint.joint_state_position = position_from_raw(raw_position);
    joint.joint_state_velocity = velocity_from_raw(raw_velocity);
    return;
  }
}

hardware_interface::return_type RotaryEncoderHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RotaryEncoderHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  elapsed_time_ += period.seconds();
  elapsed_logger_time_ += period.seconds();
  if (logger_rate_ > 0 && elapsed_logger_time_ > (1.0 / static_cast<double>(logger_rate_))) {
    elapsed_logger_time_ = 0.0;
    if (logger_state_ == 1) {
      logger_function();
    }
  }

  for (auto & joint : rotary_encoder_joints_) {
    const double curr_state_req = joint.state_request;
    if (curr_state_req < 0.0 && joint.prev_state_request >= 0.0) {
      send_state_request(joint);
    } else if (curr_state_req > 0.0) {
      joint.elapsed_state_request_time += period.seconds();
      if (joint.elapsed_state_request_time > (1.0 / curr_state_req)) {
        joint.elapsed_state_request_time = 0.0;
        send_state_request(joint);
      }
    }
    joint.prev_state_request = curr_state_req;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace rotary_encoder_ros2_control

PLUGINLIB_EXPORT_CLASS(
  rotary_encoder_ros2_control::RotaryEncoderHardwareInterface,
  hardware_interface::SystemInterface)

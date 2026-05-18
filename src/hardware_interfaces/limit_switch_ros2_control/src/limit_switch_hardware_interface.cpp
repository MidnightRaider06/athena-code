#include "limit_switch_ros2_control/limit_switch_hardware_interface.hpp"

#include <sstream>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace limit_switch_ros2_control
{

void LimitSwitchHardwareInterface::send_status_request(const LimitSwitchGPIO & gpio)
{
  CANLib::CanFrame frame;
  frame.id = can_write_id_;
  frame.dlc = 8;
  frame.data.fill(0x00);
  frame.data[0] = static_cast<uint8_t>(
    LIMIT_SWITCH_STATUS_CMD + static_cast<uint8_t>(gpio.node_id & 0xFF));
  canBus.send(frame);
}

void LimitSwitchHardwareInterface::logger_function()
{
  if (limit_switches_.empty()) {
    return;
  }

  std::ostringstream oss;
  oss << "\033[2J\033[H \nLimit Switch Logger"
      << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface_
      << " | Write CAN ID: 0x" << std::hex << std::uppercase << can_write_id_
      << " | Read CAN ID: 0x" << can_read_id_ << std::dec
      << " | HWI Update Rate: " << update_rate_
      << " | Logger Update Rate: " << logger_rate_ << "\n"
      << "Elapsed Time since first update: " << elapsed_time_ << "\n"
      << "\n--- GPIO Specific ---";

  for (const auto & gpio : limit_switches_) {
    oss << "\nGPIO: " << gpio.name
        << " | Node ID: 0x" << std::hex << std::uppercase << gpio.node_id << std::dec
        << "\n-- Commands --\n"
        << "Status Request: " << gpio.status_request
        << "\n-- State --\n"
        << "Status: " << gpio.status << "\n";
  }

  RCLCPP_INFO(rclcpp::get_logger("LimitSwitchHardwareInterface"), "%s", oss.str().c_str());
}

hardware_interface::CallbackReturn LimitSwitchHardwareInterface::on_init(
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
      rclcpp::get_logger("LimitSwitchHardwareInterface"),
      "CAN ID parameter 'can_id' is missing in hardware configuration.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    can_write_id_ = static_cast<uint32_t>(
      std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("LimitSwitchHardwareInterface"),
      "Failed to parse 'can_id': %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  can_read_id_ = can_write_id_ + 1;

  limit_switches_.clear();
  for (const auto & gpio : info_.gpios) {
    std::vector<std::string> state_if_names;
    for (const auto & si : gpio.state_interfaces) {
      state_if_names.push_back(si.name);
    }

    std::vector<std::string> command_if_names;
    for (const auto & ci : gpio.command_interfaces) {
      command_if_names.push_back(ci.name);
    }

    std::unordered_map<std::string, std::string> params_map;
    for (const auto & p : gpio.parameters) {
      params_map.emplace(p.first, p.second);
    }

    uint32_t node_id = 0;
    if (params_map.count("node_id")) {
      node_id = static_cast<uint32_t>(std::stoul(params_map.at("node_id"), nullptr, 0));
    }

    limit_switches_.push_back(LimitSwitchGPIO{
      gpio.name,
      node_id,
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

hardware_interface::CallbackReturn LimitSwitchHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!canBus.open(
      can_interface_,
      std::bind(&LimitSwitchHardwareInterface::on_can_message, this, std::placeholders::_1)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("LimitSwitchHardwareInterface"), "Failed to open CAN interface");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
LimitSwitchHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & gpio : limit_switches_) {
    for (const auto & iface : gpio.state_interface_names) {
      if (iface == "status") {
        state_interfaces.emplace_back(gpio.name, iface, &gpio.status);
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("LimitSwitchHardwareInterface"),
          "Unknown state interface '%s' for GPIO '%s'",
          iface.c_str(), gpio.name.c_str());
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
LimitSwitchHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto & gpio : limit_switches_) {
    for (const auto & iface : gpio.command_interface_names) {
      if (iface == "status_request") {
        command_interfaces.emplace_back(gpio.name, iface, &gpio.status_request);
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("LimitSwitchHardwareInterface"),
          "Unknown command interface '%s' for GPIO '%s'",
          iface.c_str(), gpio.name.c_str());
      }
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn LimitSwitchHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LimitSwitchHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LimitSwitchHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  for (auto & gpio : limit_switches_) {
    gpio.status = 0.0;
  }
  canBus.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LimitSwitchHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

void LimitSwitchHardwareInterface::on_can_message(const CANLib::CanFrame & frame)
{
  can_rx_frame_ = frame;
  if (can_rx_frame_.id != can_read_id_) {
    return;
  }

  for (auto & gpio : limit_switches_) {
    const uint8_t expected_command = static_cast<uint8_t>(
      LIMIT_SWITCH_STATUS_CMD + static_cast<uint8_t>(gpio.node_id & 0xFF));
    if (can_rx_frame_.data[0] == expected_command) {
      gpio.status = static_cast<double>(can_rx_frame_.data[1]);
      return;
    }
  }
}

hardware_interface::return_type LimitSwitchHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LimitSwitchHardwareInterface::write(
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

  for (auto & gpio : limit_switches_) {
    const double curr_status_req = gpio.status_request;
    if (curr_status_req < 0.0 && gpio.prev_status_request >= 0.0) {
      send_status_request(gpio);
    } else if (curr_status_req > 0.0) {
      gpio.elapsed_status_request_time += period.seconds();
      if (gpio.elapsed_status_request_time > (1.0 / curr_status_req)) {
        gpio.elapsed_status_request_time = 0.0;
        send_status_request(gpio);
      }
    }
    gpio.prev_status_request = curr_status_req;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace limit_switch_ros2_control

PLUGINLIB_EXPORT_CLASS(
  limit_switch_ros2_control::LimitSwitchHardwareInterface, hardware_interface::SystemInterface)

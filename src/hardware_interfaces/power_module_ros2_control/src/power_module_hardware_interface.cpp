#include "power_module_ros2_control/power_module_hardware_interface.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <sstream>

namespace power_module_ros2_control
{

void PowerModuleHardwareInterface::send_command(int can_id, int cmd_id){
  CANLib::CanFrame frame;
  frame.id  = can_id;
  frame.dlc = 2;
  frame.data.fill(0);
  frame.data[0] = cmd_id;
  frame.data[1] = CONFIRM_SEND;

  canBus.send(frame);
}

void PowerModuleHardwareInterface::logger_function()
{
  if (PowerModuleGPIOs_.empty()) {
    return;
  }

  const auto & gpio = PowerModuleGPIOs_.front();
  std::ostringstream oss;
  oss << "\033[2J\033[H \nPowerModule Logger"
      << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface_
      << " | HWI Update Rate: " << update_rate_
      << " | Logger Update Rate: " << logger_rate_ << "\n"
      << "Elapsed Time since first update: " << elapsed_time_ << "\n"
      << "\n--- GPIO Specific ---\n"
      << "GPIO: " << gpio.name << "\n"
      << "Parameters: CAN ID: 0x" << std::hex << std::uppercase << gpio.can_id << std::dec << "\n"
      << "-- Commands --\n"
      << "Kill All: " << gpio.cmd_kill_all
      << " | Kill Main: " << gpio.cmd_kill_main
      << " | Kill Jetson: " << gpio.cmd_kill_jetson
      << " | Kill By Voltage: " << gpio.cmd_kill_by_voltage << "\n"
      << "Status Request: " << gpio.status_request << "\n"
      << "-- State --\n"
      << "Kill All Sent: " << gpio.kill_all_sent
      << " | Kill Jetson Sent: " << gpio.kill_jetson_sent
      << " | Kill Main Sent: " << gpio.kill_main_sent
      << " | Kill By Voltage Sent: " << gpio.kill_by_voltage_sent << "\n"
      << " | Status: " << gpio.status << "\n";

  RCLCPP_INFO(rclcpp::get_logger("PowerModuleHardwareInterface"), "%s", oss.str().c_str());
}

hardware_interface::CallbackReturn PowerModuleHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // General HWI parameters
  can_interface_ = info_.hardware_parameters.count("can_interface") ?
    info_.hardware_parameters.at("can_interface") : "can0";
  update_rate_ = info_.hardware_parameters.count("update_rate") ?
    std::stoi(info_.hardware_parameters.at("update_rate")) : 5;
  logger_rate_ = info_.hardware_parameters.count("logger_rate") ?
    std::stoi(info_.hardware_parameters.at("logger_rate")) : 0;
  logger_state_ = info_.hardware_parameters.count("logger_state") ?
    std::stoi(info_.hardware_parameters.at("logger_state")) : 0;
  if(info_.hardware_parameters.count("can_id")) {
    try {
      can_id = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("PowerModuleHardwareInterface"), "Failed to parse 'can_id': %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("PowerModuleHardwareInterface"), "CAN ID parameter 'can_id' is missing in hardware configuration.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto& gpio : info_.gpios) {
    // Collect state interface names
    std::vector<std::string> state_if_names;
    for (const auto &si : gpio.state_interfaces) {
      state_if_names.push_back(si.name);
    }

    // Collect command interface names
    std::vector<std::string> command_if_names;
    for (const auto &ci : gpio.command_interfaces) {
      command_if_names.push_back(ci.name);
    }

    // Copy parameters into an unordered_map<string,string>
    std::unordered_map<std::string, std::string> params_map;
    for (const auto &p : gpio.parameters) {
      params_map.emplace(p.first, p.second);
    }
    
    PowerModuleGPIOs_.clear();
    PowerModuleGPIOs_.push_back(
      PowerModuleGPIO{
        .name = info_.gpios[0].name,
        .can_id = can_id,
        .kill_all_sent = 0.0,
        .kill_jetson_sent = 0.0,
        .kill_main_sent = 0.0,
        .kill_by_voltage_sent = 0.0,
        .status = 0.0,
        .cmd_kill_all = 0.0,
        .cmd_kill_jetson = 0.0,
        .cmd_kill_main = 0.0,
        .cmd_kill_by_voltage = 0.0,
        .status_request = 0.0,
        .prev_status_request = 0.0,
        .elapsed_status_request_time = 0.0,

        .state_interface_names = state_if_names,
        .command_interface_names = command_if_names,
        .parameters = params_map
      }
    );
  }

  elapsed_time_ = 0.0;
  elapsed_logger_time_ = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PowerModuleHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!canBus.open(
      can_interface_,
      std::bind(&PowerModuleHardwareInterface::on_can_message, this, std::placeholders::_1)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("PowerModuleHardwareInterface"), "Failed to open CAN interface");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

void PowerModuleHardwareInterface::on_can_message(const CANLib::CanFrame &)
{
}

std::vector<hardware_interface::StateInterface> PowerModuleHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & gpio : PowerModuleGPIOs_) {
    for (auto & iface : gpio.state_interface_names) {
      double * val = nullptr;
      if (iface == "status") {
        val = &gpio.status;
      } else if (iface == "kill_all_sent") {
        val = &gpio.kill_all_sent;
      } else if (iface == "kill_jetson_sent") {
        val = &gpio.kill_jetson_sent;
      } else if (iface == "kill_main_sent") {
        val = &gpio.kill_main_sent;
      } else if (iface == "kill_by_voltage_sent") {
        val = &gpio.kill_by_voltage_sent;
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("PowerModuleHardwareInterface"), 
          "Unknown state interface '%s' for gpio '%s'", 
          iface.c_str(), gpio.name.c_str());
        continue;
      }
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        gpio.name, iface, val));
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PowerModuleHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & gpio : PowerModuleGPIOs_) {
    for (auto & iface : gpio.command_interface_names) {
      double * val = nullptr;
      if (iface == "kill_all") {
        val = &gpio.cmd_kill_all;
      } else if (iface == "kill_jetson") {
        val = &gpio.cmd_kill_jetson;
      } else if (iface == "kill_main") {
        val = &gpio.cmd_kill_main;
      } else if (iface == "kill_by_voltage") {
        val = &gpio.cmd_kill_by_voltage;
      } else if (iface == "status_request") {
        val = &gpio.status_request;
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("PowerModuleHardwareInterface"), 
          "Unknown command interface '%s' for gpio '%s'", 
          iface.c_str(), gpio.name.c_str());
        continue;
      }
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        gpio.name, iface, val));
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn PowerModuleHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PowerModuleHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PowerModuleHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  canBus.close();
  for (auto & gpio : PowerModuleGPIOs_) {
    gpio.status = 0.0;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PowerModuleHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

hardware_interface::return_type PowerModuleHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PowerModuleHardwareInterface::write(
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

  for (auto & gpio : PowerModuleGPIOs_) {
    if (gpio.cmd_kill_all > 0.5 && gpio.kill_all_sent < 0.5) {
      send_command(gpio.can_id, CMD_KILL_ALL);
      RCLCPP_INFO(rclcpp::get_logger("PowerModuleHardwareInterface"), "Sent Kill All Command to CAN ID 0x%X", gpio.can_id);
      gpio.kill_all_sent = 1.0;
    } else if (gpio.cmd_kill_all < 0.5) {
      gpio.kill_all_sent = 0.0;
    }

    if (gpio.cmd_kill_jetson > 0.5 && gpio.kill_jetson_sent < 0.5) {
      send_command(gpio.can_id, CMD_KILL_JETSON);
      RCLCPP_INFO(rclcpp::get_logger("PowerModuleHardwareInterface"), "Sent Kill Jetson Command to CAN ID 0x%X", gpio.can_id);
      gpio.kill_jetson_sent = 1.0;
    } else if (gpio.cmd_kill_jetson < 0.5) {
      gpio.kill_jetson_sent = 0.0;
    }

    if (gpio.cmd_kill_main > 0.5 && gpio.kill_main_sent < 0.5) {
      send_command(gpio.can_id, CMD_KILL_MAIN);
      RCLCPP_INFO(rclcpp::get_logger("PowerModuleHardwareInterface"), "Sent Kill Main Command to CAN ID 0x%X", gpio.can_id);
      gpio.kill_main_sent = 1.0;
    } else if (gpio.cmd_kill_main < 0.5) {
      gpio.kill_main_sent = 0.0;
    }

    if (gpio.cmd_kill_by_voltage > 0.5 && gpio.kill_by_voltage_sent < 0.5) {
      send_command(gpio.can_id, CMD_KILL_BY_VOLTAGE);
      RCLCPP_INFO(rclcpp::get_logger("PowerModuleHardwareInterface"), "Sent Kill By Voltage Command to CAN ID 0x%X", gpio.can_id);
      gpio.kill_by_voltage_sent = 1.0;
    } else if (gpio.cmd_kill_by_voltage < 0.5) {
      gpio.kill_by_voltage_sent = 0.0;
    }

    gpio.status = (static_cast<uint8_t>(gpio.kill_all_sent) << 3) | 
                  (static_cast<uint8_t>(gpio.cmd_kill_jetson) << 2) | 
                  (static_cast<uint8_t>(gpio.cmd_kill_main) << 1) | 
                  static_cast<uint8_t>(gpio.cmd_kill_by_voltage);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace power_module_ros2_control

PLUGINLIB_EXPORT_CLASS(
  power_module_ros2_control::PowerModuleHardwareInterface,
  hardware_interface::SystemInterface)

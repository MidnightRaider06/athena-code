#include "led_ros2_control/led_hardware_interface.hpp"

#include <fcntl.h>
#include <sstream>
#include <unistd.h>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <iomanip>
#include <cmath>

namespace led_ros2_control
{

void LEDHardwareInterface::send_command(int can_id, int cmd_id){
  CANLib::CanFrame frame;
  frame.id  = can_id;
  frame.dlc = 2;
  frame.data.fill(0);
  frame.data[0] = cmd_id;
  frame.data[1] = CONFIRM_SEND;

  canBus.send(frame);
}

void LEDHardwareInterface::logger_function()
{
  if (LEDGPIOs_.empty()) {
    return;
  }

  for (const auto & gpio : LEDGPIOs_) {
    std::ostringstream oss;
    oss << "\033[2J\033[H \nLED Logger"
      << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface_
      << " | CAN ID: 0x" << std::hex << std::uppercase << can_id_ << std::dec
      << " | HWI Update Rate: " << update_rate_
      << " | Logger Update Rate: " << logger_rate_ << "\n"
      << "Elapsed Time since first update: " << elapsed_time_ << "\n"
      << "\n--- GPIO Specific ---\n"
      << "GPIO: " << gpio.name << " | Node ID: 0x" << std::hex << std::uppercase << gpio.node_id << std::dec << "\n"
      << "-- Commands --\n"
      << "Red Command: " << gpio.red_command
      << " | Green Command: " << gpio.green_command
      << " | Blue Command: " << gpio.blue_command
      << " | Status Request: " << gpio.status_request << "\n"
      << "-- State --\n"
      << "Status: " << gpio.status << "\n";

    RCLCPP_INFO(rclcpp::get_logger("LEDHardwareInterface"), "%s", oss.str().c_str());
  }
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_init(
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
  if (info_.hardware_parameters.count("can_id")) {
    try {
      can_id_ = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("LEDHardwareInterface"), "Failed to parse 'can_id': %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("LEDHardwareInterface"), "CAN ID parameter 'can_id' is missing in hardware configuration.");
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
    // Determine whether this gpio is rgb from its parameters
    bool is_rgb_flag = false;
    if (params_map.count("is_rgb")) {
      const std::string &v = params_map.at("is_rgb");
      is_rgb_flag = (v == "true" || v == "1");
    }
    // parse per-gpio node_id if provided
    uint32_t gpio_node_id = 0;
    if (params_map.count("node_id")) {
      try {
        gpio_node_id = static_cast<uint32_t>(std::stoul(params_map.at("node_id")));
      } catch (const std::exception & e) {
        RCLCPP_WARN(rclcpp::get_logger("LEDHardwareInterface"), "Failed to parse gpio node_id for '%s': %s", gpio.name.c_str(), e.what());
      }
    }

    LEDGPIOs_.push_back(
      LEDGPIO{
        .name = gpio.name,
        .is_rgb = is_rgb_flag,
        .node_id = gpio_node_id,
        .status = 0.0,
        .intensity = 0.0,
        .red_command = 0.0,
        .green_command = 0.0,
        .blue_command = 0.0,
        .status_request = 0.0,
        .prev_status_request = 0.0,
        .elapsed_status_request_time = 0.0,
        .prev_intensity = std::numeric_limits<double>::quiet_NaN(),
        .prev_red_command = std::numeric_limits<double>::quiet_NaN(),
        .prev_green_command = std::numeric_limits<double>::quiet_NaN(),
        .prev_blue_command = std::numeric_limits<double>::quiet_NaN(),

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

hardware_interface::CallbackReturn LEDHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LEDHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & gpio : LEDGPIOs_) {
    for (auto & iface : gpio.state_interface_names) {
      double * val = nullptr;
      if (iface == "status") {
        val = &gpio.status;
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("LEDHardwareInterface"), 
          "Unknown state interface '%s' for GPIO '%s'", 
          iface.c_str(), gpio.name.c_str());
        continue;
      }
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        gpio.name, iface, val));
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LEDHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & gpio : LEDGPIOs_) {
    for (auto & iface : gpio.command_interface_names) {
      double * val = nullptr;
      // Only export RGB interfaces when is_rgb is true; otherwise export intensity
      if (gpio.is_rgb) {
        if (iface == "red") {
          val = &gpio.red_command;
        } else if (iface == "green") {
          val = &gpio.green_command;
        } else if (iface == "blue") {
          val = &gpio.blue_command;
        } else if (iface == "status_request") {
          val = &gpio.status_request;
        } else {
          // skip intensity when rgb mode
          continue;
        }
      } else {
        if (iface == "intensity") {
          val = &gpio.intensity;
        } else if (iface == "status_request") {
          val = &gpio.status_request;
        } else {
          // skip rgb channels when not rgb
          continue;
        }
      }
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        gpio.name, iface, val));
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  for (auto & gpio : LEDGPIOs_) {
    gpio.status = 0.0;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

hardware_interface::return_type LEDHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LEDHardwareInterface::write(
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

  for (auto & gpio : LEDGPIOs_) {
    if (gpio.status_request < 0.0 && gpio.prev_status_request >= 0.0) {
    } else if (gpio.status_request > 0.0) {
      gpio.elapsed_status_request_time += period.seconds();
      if (gpio.elapsed_status_request_time > (1.0 / gpio.status_request)) {
        gpio.elapsed_status_request_time = 0.0;
      }
    }
    gpio.prev_status_request = gpio.status_request;
    // Build and send CAN frames for LED commands
    CANLib::CanFrame frame;
    frame.id = can_id_;
    frame.dlc = 8;
    frame.data.fill(0);

    if (gpio.is_rgb) {
      // RGB mode: send when any channel changed
      const double r = gpio.red_command;
      const double g = gpio.green_command;
      const double b = gpio.blue_command;
      const bool changed = (std::isnan(gpio.prev_red_command) || r != gpio.prev_red_command) ||
                          (std::isnan(gpio.prev_green_command) || g != gpio.prev_green_command) ||
                          (std::isnan(gpio.prev_blue_command) || b != gpio.prev_blue_command);
      if (changed) {
        frame.data[0] = static_cast<uint8_t>(CMD_RGB + static_cast<uint8_t>(gpio.node_id & 0xFF));
        frame.data[1] = std::clamp(static_cast<int>(std::round(r)), 0, 255);
        frame.data[2] = std::clamp(static_cast<int>(std::round(g)), 0, 255);
        frame.data[3] = std::clamp(static_cast<int>(std::round(b)), 0, 255);
        canBus.send(frame);
        gpio.prev_red_command = r;
        gpio.prev_green_command = g;
        gpio.prev_blue_command = b;
        RCLCPP_INFO(rclcpp::get_logger("LEDHardwareInterface"), "Sent RGB frame to node 0x%X (CAN ID 0x%X)", gpio.node_id, can_id_);
      }
    } else {
      const double intensity = gpio.intensity;
      const bool changed = std::isnan(gpio.prev_intensity) || intensity != gpio.prev_intensity;
      if (changed) {
        frame.data[0] = static_cast<uint8_t>(CMD_INTENSITY + static_cast<uint8_t>(gpio.node_id & 0xFF));
        frame.data[1] = std::clamp(static_cast<int>(std::round(intensity)), 0, 255);
        canBus.send(frame);
        gpio.prev_intensity = intensity;
        RCLCPP_INFO(rclcpp::get_logger("LEDHardwareInterface"), "Sent intensity frame to node 0x%X (CAN ID 0x%X val=%d)", gpio.node_id, can_id_, frame.data[1]);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace led_ros2_control

PLUGINLIB_EXPORT_CLASS(
  led_ros2_control::LEDHardwareInterface, hardware_interface::SystemInterface)

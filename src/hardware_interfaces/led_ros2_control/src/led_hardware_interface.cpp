// Copyright (c) 2024 UMD Loop
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include "led_ros2_control/led_hardware_interface.hpp"

#include <fcntl.h>
#include <sstream>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace led_ros2_control
{

// GPIO utility functions implementation
namespace gpio_utils
{

int setup_gpio_output(int pin)
{
  // Export GPIO
  int fd = ::open("/sys/class/gpio/export", O_WRONLY);
  if (fd >= 0) {
    std::string pin_str = std::to_string(pin);
    ::write(fd, pin_str.c_str(), pin_str.length());
    ::close(fd);
    usleep(100000);  // Wait for GPIO to be exported
  }

  // Set direction to output
  std::stringstream direction_path;
  direction_path << "/sys/class/gpio/gpio" << pin << "/direction";
  fd = ::open(direction_path.str().c_str(), O_WRONLY);
  if (fd < 0) {
    return -1;
  }
  ::write(fd, "out", 3);
  ::close(fd);

  // Open value file
  std::stringstream value_path;
  value_path << "/sys/class/gpio/gpio" << pin << "/value";
  int value_fd = ::open(value_path.str().c_str(), O_RDWR);
  
  return value_fd;
}

void cleanup_gpio(int pin, int fd)
{
  if (fd >= 0) {
    ::close(fd);
  }

  // Unexport GPIO
  int export_fd = ::open("/sys/class/gpio/unexport", O_WRONLY);
  if (export_fd >= 0) {
    std::string pin_str = std::to_string(pin);
    ::write(export_fd, pin_str.c_str(), pin_str.length());
    ::close(export_fd);
  }
}

bool write_gpio(int fd, bool value)
{
  if (fd < 0) return false;
  
  char val = value ? '1' : '0';
  lseek(fd, 0, SEEK_SET);
  return (::write(fd, &val, 1) == 1);
}

}  // namespace gpio_utils

// Hardware Interface Implementation

hardware_interface::CallbackReturn LEDHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != 
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse hardware parameters
  if (!info_.hardware_parameters.count("gpio_pin")) {
    RCLCPP_ERROR(
      rclcpp::get_logger("LEDHardwareInterface"),
      "gpio_pin parameter is required");
    return hardware_interface::CallbackReturn::ERROR;
  }
  gpio_pin_ = std::stoi(info_.hardware_parameters.at("gpio_pin"));

  // Default state
  if (info_.hardware_parameters.count("default_state")) {
    default_state_ = (info_.hardware_parameters.at("default_state") == "on");
  } else {
    default_state_ = false;  // OFF by default
  }

  // Initialize state variables
  led_state_ = default_state_ ? 1.0 : 0.0;
  is_connected_ = 0.0;

  // Initialize command variables
  led_command_ = default_state_ ? 1.0 : 0.0;

  gpio_fd_ = -1;
  hw_connected_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "Initialized LED on GPIO pin %d (default: %s)",
    gpio_pin_, default_state_ ? "ON" : "OFF");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "Configuring LED hardware...");

  // Setup GPIO output
  gpio_fd_ = gpio_utils::setup_gpio_output(gpio_pin_);
  if (gpio_fd_ < 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("LEDHardwareInterface"),
      "Failed to setup GPIO output on pin %d - running in SIMULATION mode", gpio_pin_);
    hw_connected_ = false;  // Mark as simulation mode
  } else {
    hw_connected_ = true;
    // Set initial state on real hardware
    gpio_utils::write_gpio(gpio_fd_, default_state_);
    RCLCPP_INFO(
      rclcpp::get_logger("LEDHardwareInterface"),
      "Successfully configured LED hardware (GPIO mode)");
  }

  is_connected_ = hw_connected_ ? 1.0 : 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "LED hardware configured (%s)", hw_connected_ ? "REAL GPIO" : "SIMULATION");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> 
LEDHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Use the gpio name from URDF
  const std::string& name = info_.gpios[0].name;

  // LED state
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "led_state", &led_state_));

  // Connection status
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "is_connected", &is_connected_));

  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "Exported %zu state interfaces", state_interfaces.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> 
LEDHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Use the gpio name from URDF
  const std::string& name = info_.gpios[0].name;

  // LED command
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(name, "led_command", &led_command_));

  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "Exported %zu command interfaces", command_interfaces.size());

  return command_interfaces;
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "Activating LED hardware...");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "Deactivating LED hardware...");

  // Turn LED off on deactivation (safety)
  if (hw_connected_ && gpio_fd_ >= 0) {
    gpio_utils::write_gpio(gpio_fd_, false);
    led_state_ = 0.0;
    
    RCLCPP_INFO(
      rclcpp::get_logger("LEDHardwareInterface"),
      "LED turned OFF (deactivation)");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "Cleaning up LED hardware...");

  // Ensure LED is OFF before cleanup
  if (gpio_fd_ >= 0) {
    gpio_utils::write_gpio(gpio_fd_, false);
  }

  // Cleanup GPIO - unexport and close file descriptor
  if (gpio_fd_ >= 0) {
    gpio_utils::cleanup_gpio(gpio_pin_, gpio_fd_);
    gpio_fd_ = -1;
  }

  hw_connected_ = false;
  is_connected_ = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "LED hardware cleanup complete");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LEDHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LEDHardwareInterface"),
    "Shutting down LED hardware...");

  // Delegate to cleanup to release resources
  return on_cleanup(previous_state);
}

hardware_interface::return_type LEDHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // LED state is known from commands, no need to read from GPIO
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LEDHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // Check if LED command changed
  bool commanded_on = (led_command_ > 0.5);
  bool currently_on = (led_state_ > 0.5);

  if (commanded_on != currently_on) {
    // Write to real GPIO if available, otherwise simulate
    if (hw_connected_ && gpio_fd_ >= 0) {
      gpio_utils::write_gpio(gpio_fd_, commanded_on);
    }
    
    led_state_ = commanded_on ? 1.0 : 0.0;
    
    RCLCPP_DEBUG(
      rclcpp::get_logger("LEDHardwareInterface"),
      "LED turned %s%s", commanded_on ? "ON" : "OFF",
      hw_connected_ ? "" : " (simulated)");
  }

  return hardware_interface::return_type::OK;
}

}  // namespace led_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  led_ros2_control::LEDHardwareInterface,
  hardware_interface::SystemInterface)


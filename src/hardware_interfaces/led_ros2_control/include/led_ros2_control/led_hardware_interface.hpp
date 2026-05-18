#ifndef LED_ROS2_CONTROL__LED_HARDWARE_INTERFACE_HPP_
#define LED_ROS2_CONTROL__LED_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <limits>
#include <algorithm>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "umdloop_can_library/SocketCanBus.hpp"
#include "umdloop_can_library/CanFrame.hpp"

namespace CANLib 
{
  struct CanFrame;
}

namespace led_ros2_control
{

class LEDHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LEDHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // -- Helper Functions --
  void send_command(int can_id, int cmd_id);
  void logger_function();

private:
  struct LEDGPIO
  {
    std::string name;
    bool is_rgb;
    uint32_t node_id;
    double status;
    double intensity;
    double red_command;
    double green_command;
    double blue_command;
    double status_request;
    double prev_status_request;
    double elapsed_status_request_time;
    double prev_intensity;
    double prev_red_command;
    double prev_green_command;
    double prev_blue_command;

    std::vector<std::string> state_interface_names;
    std::vector<std::string> command_interface_names;
    std::unordered_map<std::string, std::string> parameters;
  };

  std::string can_interface_;
  CANLib::SocketCanBus canBus;
  CANLib::CanFrame can_tx_frame_;

  // CAN bus ID (constant for this HWI)
  uint32_t can_id_ = 0;

  std::vector<LEDGPIO> LEDGPIOs_;
  int update_rate_;
  int logger_rate_;
  int logger_state_;
  double elapsed_time_;
  double elapsed_logger_time_;

  // Multiplexor byte
  static constexpr uint8_t CMD_INTENSITY = 0x20;
  static constexpr uint8_t CMD_RGB = 0x30;

  // Parameter to confirm whether to send command or not via CAN
  static constexpr uint8_t DECLINE_SEND = 0;
  static constexpr uint8_t CONFIRM_SEND = 1;
};

}  // namespace led_ros2_control

#endif  // LED_ROS2_CONTROL__LED_HARDWARE_INTERFACE_HPP_

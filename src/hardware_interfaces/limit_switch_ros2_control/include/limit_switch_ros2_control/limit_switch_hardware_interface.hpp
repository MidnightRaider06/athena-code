#ifndef LIMIT_SWITCH_ROS2_CONTROL__LIMIT_SWITCH_HARDWARE_INTERFACE_HPP_
#define LIMIT_SWITCH_ROS2_CONTROL__LIMIT_SWITCH_HARDWARE_INTERFACE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "umdloop_can_library/CanFrame.hpp"
#include "umdloop_can_library/SocketCanBus.hpp"

namespace CANLib
{
struct CanFrame;
}

namespace limit_switch_ros2_control
{

class LimitSwitchHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LimitSwitchHardwareInterface)

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

  void on_can_message(const CANLib::CanFrame & frame);
  void logger_function();

private:
  struct LimitSwitchGPIO
  {
    std::string name;
    uint32_t node_id;
    double status;
    double status_request;
    double prev_status_request;
    double elapsed_status_request_time;

    std::vector<std::string> state_interface_names;
    std::vector<std::string> command_interface_names;
    std::unordered_map<std::string, std::string> parameters;
  };

  void send_status_request(const LimitSwitchGPIO & gpio);

  std::string can_interface_;
  CANLib::SocketCanBus canBus;
  CANLib::CanFrame can_rx_frame_;

  uint32_t can_write_id_ = 0;
  uint32_t can_read_id_ = 0;
  int update_rate_ = 0;
  int logger_rate_ = 0;
  int logger_state_ = 0;
  double elapsed_time_ = 0.0;
  double elapsed_logger_time_ = 0.0;

  std::vector<LimitSwitchGPIO> limit_switches_;

  static constexpr uint8_t LIMIT_SWITCH_STATUS_CMD = 0x20;
};

}  // namespace limit_switch_ros2_control

#endif  // LIMIT_SWITCH_ROS2_CONTROL__LIMIT_SWITCH_HARDWARE_INTERFACE_HPP_

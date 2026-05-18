#ifndef POWER_MODULE_ROS2_CONTROL__POWER_MODULE_HARDWARE_INTERFACE_HPP_
#define POWER_MODULE_ROS2_CONTROL__POWER_MODULE_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
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

namespace power_module_ros2_control
{

class PowerModuleHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PowerModuleHardwareInterface)

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
  void on_can_message(const CANLib::CanFrame & frame);
  void send_command(int can_id, int cmd_id);
  void logger_function();

private:
  struct PowerModuleGPIO
  {
    std::string name;
    uint32_t can_id;
    double kill_all_sent;
    double kill_jetson_sent;
    double kill_main_sent;
    double kill_by_voltage_sent;
    double status;
    double cmd_kill_all;
    double cmd_kill_jetson;
    double cmd_kill_main;
    double cmd_kill_by_voltage;
    double status_request;
    double prev_status_request;
    double elapsed_status_request_time;

    std::vector<std::string> state_interface_names;
    std::vector<std::string> command_interface_names;
    std::unordered_map<std::string, std::string> parameters;
  };

  std::string can_interface_;
  uint32_t can_id;
  CANLib::SocketCanBus canBus;
  CANLib::CanFrame can_tx_frame_;
  std::vector<PowerModuleGPIO> PowerModuleGPIOs_;
  int update_rate_;
  int logger_rate_;
  int logger_state_;
  double elapsed_time_;
  double elapsed_logger_time_;

  // Multiplexor byte
  static constexpr uint8_t CMD_KILL_ALL = 0x12;
  static constexpr uint8_t CMD_KILL_JETSON = 0x13;
  static constexpr uint8_t CMD_KILL_MAIN = 0x14;
  static constexpr uint8_t CMD_KILL_BY_VOLTAGE = 0x15;

  // Parameter to confirm whether to send command or not via CAN
  static constexpr uint8_t DECLINE_SEND = 0;
  static constexpr uint8_t CONFIRM_SEND = 1;
};

}  // namespace power_module_ros2_control

#endif  // POWER_MODULE_ROS2_CONTROL__POWER_MODULE_HARDWARE_INTERFACE_HPP_

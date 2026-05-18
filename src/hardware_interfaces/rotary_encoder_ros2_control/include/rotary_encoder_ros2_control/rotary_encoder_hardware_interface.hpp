#ifndef ROTARY_ENCODER_ROS2_CONTROL__ROTARY_ENCODER_HARDWARE_INTERFACE_HPP_
#define ROTARY_ENCODER_ROS2_CONTROL__ROTARY_ENCODER_HARDWARE_INTERFACE_HPP_

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

namespace rotary_encoder_ros2_control
{

class RotaryEncoderHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RotaryEncoderHardwareInterface)

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
  struct RotaryEncoderJoint
  {
    std::string name;
    uint32_t node_id;
    uint8_t rotary_encoder_id;
    double joint_state_position;
    double joint_state_velocity;
    double state_request;
    double prev_state_request;
    double elapsed_state_request_time;

    std::vector<std::string> state_interface_names;
    std::vector<std::string> command_interface_names;
    std::unordered_map<std::string, std::string> parameters;
  };

  void send_state_request(const RotaryEncoderJoint & joint);
  static double position_from_raw(int16_t raw_position);
  static double velocity_from_raw(int16_t raw_velocity);

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

  std::vector<RotaryEncoderJoint> rotary_encoder_joints_;

  static constexpr uint8_t ROTARY_ENCODER_STATE_CMD = 0x20;
};

}  // namespace rotary_encoder_ros2_control

#endif  // ROTARY_ENCODER_ROS2_CONTROL__ROTARY_ENCODER_HARDWARE_INTERFACE_HPP_

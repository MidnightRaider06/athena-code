#ifndef TALON_HARDWARE_INTERACE_HPP_
#define TALON_HARDWARE_INTERACE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "talon_ros2_control/talon_control.h"

namespace talon_ros2_control
{

class TALONHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TALONHardwareInterface)

  TALONHardwareInterface();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void logger_function();
  void enable_system_thread();

private:
  enum class integration_level_t : std::uint8_t
  {
    UNDEFINED = 0,
    POSITION = 1,
    VELOCITY = 2,
  };

  enum class joint_type_t : std::uint8_t
  {
    REVOLUTE = 0,
    PRISMATIC = 1,
  };

  struct TalonJoint
  {
    std::string name;
    int node_id;
    joint_type_t joint_type;
    double max_disp;
    MotorConfig motor_config;
    TalonSRX * motor;
    integration_level_t control_level;

    double joint_state_position;
    double joint_state_velocity;
    double motor_temperature;
    double motor_torque_current;
    double motor_status;

    double joint_command_position;
    double joint_command_velocity;
    double motor_status_req;
    double prev_status_req;
    double elapsed_status_request_time;

    std::vector<std::string> state_interface_names;
    std::vector<std::string> command_interface_names;
    std::unordered_map<std::string, std::string> parameters;
  };

  int update_rate;
  double elapsed_update_time;
  double elapsed_time;
  double elapsed_logger_time;
  int logger_rate;
  int logger_state;
  int num_joints;

  std::string can_interface;
  std::vector<TalonJoint> TALONJoints_;
  std::thread worker;
  std::atomic<bool> is_running{false};
};

}  // namespace talon_ros2_control

#endif  // TALON_HARDWARE_INTERACE_HPP_

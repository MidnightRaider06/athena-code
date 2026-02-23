#ifndef TALON_HARDWARE_INTERACE_HPP_
#define TALON_HARDWARE_INTERACE_HPP_

#include <netinet/in.h>
#include <memory>
#include <string>
#include <vector>

#include <atomic>
#include <thread>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

#include "talon_ros2_control/talon_control.h"

namespace talon_ros2_control
{
class TALONHardwareInterface : public hardware_interface::SystemInterface // Inheriting from System Interface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TALONHardwareInterface)

  TALONHardwareInterface();

  // Initialization, so reading parameters, initializing variables, checking if all the joint state and command interfaces are correct
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // Exports/exposes Interfaces that are available so that the controllers
  // know what to read and write to
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Lifecycle
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
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces
  ) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Helper Functions
  void logger_function();
  void enable_system_thread();

private:
  // Hardware Interface Parameters
  int update_rate;
  double elapsed_update_time; // Time since last hardware interface update
  double elapsed_time; // Time since first hardware interface update
  double elapsed_logger_time; // Time since last logger update
  int logger_rate; // Logger update rate
  int logger_state; // Logger on/off state

  // Keeps track of amount of joints
  int num_joints;

  // Maximum displacement for prismatic joints
  std::vector<double> max_disp;

  // Store the state for the simulated robot
  std::vector<double> joint_state_position_;
  std::vector<double> joint_state_velocity_;
  
  // Store the command for the simulated robot
  std::vector<double> joint_command_position_;
  std::vector<double> joint_command_velocity_;

  // Telemetry data
  std::vector<double> motor_temperature_;    // Motor temperature in °C
  std::vector<double> motor_torque_current_; // Motor output current in A

  // Talon specific information
  std::vector<int> joint_node_ids;
  std::string can_interface;
  std::vector<TalonSRX*> talon_motors;
  std::vector<MotorConfig> motor_configs_;
  std::thread worker;
  std::atomic<bool> is_running = false;

  // Modes for control mode
  enum integration_level_t : std::uint8_t
  {
    UNDEFINED = 0,
    POSITION = 1,
    VELOCITY = 2,
  };

  // Active control mode
  std::vector<integration_level_t> control_level_;

  // Types of joints
  enum class joint_type_t : std::uint8_t
  {
    REVOLUTE = 0,
    PRISMATIC = 1,
  };

  // Type of joint for each actuator
  std::vector<joint_type_t> joint_type_;
};

}  // namespace talon_hardware_interface

#endif  // TALON_HARDWARE_INTERACE_HPP_


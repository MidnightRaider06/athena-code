#ifndef ATHENA_ARM_CONTROLLERS__MANUAL_ARM_JOINT_BY_JOINT_CONTROLLER_HPP_
#define ATHENA_ARM_CONTROLLERS__MANUAL_ARM_JOINT_BY_JOINT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include <arm_controllers/manual_arm_joint_by_joint_controller_parameters.hpp>
#include "athena_arm_controllers/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_srvs/srv/set_bool.hpp"

// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "sensor_msgs/msg/joy.hpp"


namespace arm_controllers
{
// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = 0;

// amount of joystick axes
static constexpr int joystick_axes = 6;

// amount of joystick buttons
static constexpr int joystick_buttons = 13;

// TODO(anyone: example setup for control mode (usually you will use some enums defined in messages)
enum class control_mode_type : std::uint8_t
{
  FAST = 0,
  SLOW = 1,
};

class ManualArmJointByJointController : public controller_interface::ControllerInterface
{
public:
  ATHENA_ARM_CONTROLLERS__VISIBILITY_PUBLIC
  ManualArmJointByJointController();

  ATHENA_ARM_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  ATHENA_ARM_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  ATHENA_ARM_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ATHENA_ARM_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ATHENA_ARM_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ATHENA_ARM_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ATHENA_ARM_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // TODO(anyone): replace the state and command message types
  using ControllerReferenceMsg = sensor_msgs::msg::Joy;
  using ControllerModeSrvType = std_srvs::srv::SetBool;
  using ControllerStateMsg = control_msgs::msg::JointControllerState;

protected:
  std::shared_ptr<manual_arm_joint_by_joint_controller::ParamListener> param_listener_;
  manual_arm_joint_by_joint_controller::Params params_;

  int num_joints;
  bool actuator_active_ = false;
  double actuator_iterator = 0.0;

  // Represents velocities of each joint
  std::vector<double> joint_velocities_;

  // Represents maximum velocities of each joint
  std::vector<double> max_velocities_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  rclcpp::Service<ControllerModeSrvType>::SharedPtr set_slow_control_mode_service_;
  realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

private:
  // callback for topic interface
  ATHENA_ARM_CONTROLLERS__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
};

}  // namespace arm_controllers

#endif  // ATHENA_ARM_CONTROLLERS__MANUAL_ARM_JOINT_BY_JOINT_CONTROLLER_HPP_

#include "athena_arm_controllers/manual_3dof_wrist_joint_by_joint_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

using ControllerReferenceMsg = arm_controllers::Manual3DOFWristJointByJointController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  std::shared_ptr<ControllerReferenceMsg> & msg, const int & axes_count, const int & button_count)
{
  msg->axes.resize(axes_count, std::numeric_limits<double>::quiet_NaN());
  msg->buttons.resize(button_count, std::numeric_limits<int32_t>::quiet_NaN());
}

}  // namespace

namespace arm_controllers
{
Manual3DOFWristJointByJointController::Manual3DOFWristJointByJointController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn Manual3DOFWristJointByJointController::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);

  try
  {
    param_listener_ = std::make_shared<manual_3dof_wrist_joint_by_joint_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Manual3DOFWristJointByJointController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  num_joints = static_cast<int>(params_.joints.size());
  joint_velocities_.resize(num_joints, 0.0); // Output
  max_velocities_ = params_.joint_max_velocities;

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "controller_input", subscribers_qos,
    std::bind(&Manual3DOFWristJointByJointController::reference_callback, this, std::placeholders::_1));
  
  // Create, populate with NaN, and write message to input_ref_ to be used in reference callback
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, joystick_axes, joystick_buttons);
  input_ref_.writeFromNonRT(msg);

  auto set_slow_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response)
  {
    if (request->data)
    {
      control_mode_.writeFromNonRT(control_mode_type::SLOW);
    }
    else
    {
      control_mode_.writeFromNonRT(control_mode_type::FAST);
    }
    response->success = true;
  };

  set_slow_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_slow_control_mode", set_slow_mode_service_callback,
    rmw_qos_profile_services_hist_keep_all);

  try
  {
    // State publisher
    s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // TODO(anyone): Reserve memory in state publisher depending on the message type
  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = params_.joints[0];
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void Manual3DOFWristJointByJointController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  input_ref_.writeFromNonRT(msg);
}

controller_interface::InterfaceConfiguration Manual3DOFWristJointByJointController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(num_joints);

  for (const auto & joint : params_.joints)
  {
    command_interfaces_config.names.push_back(joint + "/velocity");
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration Manual3DOFWristJointByJointController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(num_joints);
  for (const auto & joint : params_.joints) {
    state_interfaces_config.names.push_back(joint + "/velocity");
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn Manual3DOFWristJointByJointController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT)(), joystick_axes, joystick_buttons);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Manual3DOFWristJointByJointController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Manual3DOFWristJointByJointController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto current_ref = input_ref_.readFromRT();

  if (!std::isnan((*current_ref)->axes[0]))
  {
    // Motor A: U/D on Left Joystick AND O button 
    joint_velocities_[0] = -(*current_ref)->axes[1] * static_cast<float>((*current_ref)->buttons[1]) * max_velocities_[0];
    
    // Motor B: L/R on Left Joystick AND O button
    joint_velocities_[1] = -(*current_ref)->axes[0] * static_cast<float>((*current_ref)->buttons[1]) * max_velocities_[1];

    // Motor C: U/D on Right Joystick AND O button
    joint_velocities_[2] = -(*current_ref)->axes[3] * static_cast<float>((*current_ref)->buttons[1]) * max_velocities_[2];
  }
  else{
    // RCLCPP_INFO(get_node()->get_logger(), "Returning NaN");
    joint_velocities_.resize(num_joints, 0.0);
  }

  // RCLCPP_INFO(get_node()->get_logger(), "Size of Command Interface: %d", command_interfaces_.size());

  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    if (*(control_mode_.readFromRT()) == control_mode_type::SLOW)
    {
      joint_velocities_[i] /= 2;
    }
    // RCLCPP_INFO(get_node()->get_logger(), "Joint %d: %f", i, joint_velocities_[i]);
    command_interfaces_[i].set_value(joint_velocities_[i]);

    // (*current_ref)->axes[i] = std::numeric_limits<double>::quiet_NaN();
    // (*current_ref)->buttons[i] = std::numeric_limits<double>::quiet_NaN();
  }

  if (state_publisher_ && state_publisher_->trylock())
  {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.set_point = command_interfaces_[CMD_MY_ITFS].get_value();
    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace arm_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arm_controllers::Manual3DOFWristJointByJointController, controller_interface::ControllerInterface)

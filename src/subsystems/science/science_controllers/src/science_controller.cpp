// Copyright (c) 2025, UMDLoop
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "science_controllers/science_controller.hpp"
#include "science_controllers/science_manual_parameters.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

#include "controller_interface/helpers.hpp"

#define DEBUG_MODE 0

namespace
{

using ControllerReferenceMsg = science_controllers::ScienceManual::ControllerReferenceMsg;

// Utility: reset joystick msg for RT buffer
void reset_controller_reference_msg(
  std::shared_ptr<ControllerReferenceMsg> & msg, const int & axes_count, const int & button_count)
{
  msg->axes.resize(axes_count, 0.0);
  msg->buttons.resize(button_count, 0.0);
}
}  // namespace

namespace science_controllers
{

ScienceManual::ScienceManual() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn ScienceManual::on_init()
{
  control_mode_.initRT(control_mode_type::STAGE1);

  servo_scoop_a_toggle = false;
  servo_scoop_b_counter = 0; // TESTING

  try {
    param_listener_ = std::make_shared<science_manual::ParamListener>(get_node());
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during controller init: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn ScienceManual::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  // Set default STAGE 1
  current_mode_ = control_mode_type::STAGE1;

  // Fill each joint variable
  pump_a = params_.pump_a;
  pump_b = params_.pump_b;
  lift_rack_and_pinion_l = params_.lift_rack_and_pinion_l;
  lift_rack_and_pinion_r = params_.lift_rack_and_pinion_r;
  scoop_servos = params_.scoop_servos;
  scoop_spinner = params_.scoop_spinner;
  sampler_lift_l = params_.sampler_lift_l;
  sampler_lift_r = params_.sampler_lift_r;
  auger_spinner = params_.auger_spinner;
  auger_lift = params_.auger_lift;

  // Fill composite joint groups
  stepper_pump_joints_.clear();
  stepper_pump_joints_ = {
    params_.pump_a,
    params_.pump_b
  };

  dc_joints_.clear();
  dc_joints_ = {
    params_.scoop_spinner,
    params_.auger_spinner
  }; 

  servo_joints_ = params_.scoop_servos;
  servo_joints_.push_back(params_.sampler_lift_l);
  servo_joints_.push_back(params_.sampler_lift_r);
  servo_joints_.push_back(params_.auger_lift);

  rack_pinion_joints_.clear();
  rack_pinion_joints_.push_back(params_.lift_rack_and_pinion_l);
  rack_pinion_joints_.push_back(params_.lift_rack_and_pinion_r);

  // Populate joints vector
  joints_.clear();

  joints_.push_back(pump_a);
  joints_.push_back(pump_b);
  joints_.push_back(lift_rack_and_pinion_l);
  joints_.push_back(lift_rack_and_pinion_r);

  joints_.insert(joints_.end(), scoop_servos.begin(), scoop_servos.end());

  joints_.push_back(scoop_spinner);
  joints_.push_back(sampler_lift_l);
  joints_.push_back(sampler_lift_r);
  joints_.push_back(auger_spinner);
  joints_.push_back(auger_lift);

  if (!params_.state_joints.empty()) {
    state_joints_ = params_.state_joints;
  } else {
    state_joints_ = joints_;
  }

  if (joints_.size() != state_joints_.size()) {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'joints' (%zu) and 'state_joints' (%zu) must be equal!",
      joints_.size(), state_joints_.size());
    return CallbackReturn::FAILURE;
  }

  prev_buttons_.assign(joystick_buttons, 0);

  // Start the Rack and Pinions off at their intended "0" position
  // rack_left_position = params_.position_range_lift_left[1];
  // rack_right_position = params_.position_range_lift_right[0]; // Orientation is flipped, so we start from top

  rack_left_position = 185;
  rack_right_position = 185;

  // QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "/science_manual", subscribers_qos,
    std::bind(&ScienceManual::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, joystick_axes, joystick_buttons);
  input_ref_.writeFromNonRT(msg);

  // State publisher
  s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
    "/science_manual/state", rclcpp::QoS(rclcpp::KeepLast(1)));
  state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);

  if (state_publisher_ && state_publisher_->trylock()) {
    state_publisher_->msg_.header.frame_id =
      (!joints_.empty() ? joints_[0] : std::string("base"));
    state_publisher_->msg_.set_point = 0.0;
    state_publisher_->msg_.process_value = 0.0;
    state_publisher_->msg_.command = 0.0;
    state_publisher_->unlockAndPublish();
  }

  RCLCPP_INFO(get_node()->get_logger(), "Configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void ScienceManual::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (prev_buttons_.empty()) {
    prev_buttons_.resize(msg->buttons.size(), 0);
  }
  input_ref_.writeFromNonRT(msg);
}

controller_interface::InterfaceConfiguration ScienceManual::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Pumps
  for (const auto & joint : stepper_pump_joints_)
  {
    cfg.names.push_back(joint + "/velocity");
  }

  // Lift
  for (const auto & joint : rack_pinion_joints_) {
    cfg.names.push_back(joint + "/position");
  }

  // Scoops
  cfg.names.push_back(params_.scoop_spinner + "/velocity");
  for (const auto & joint : scoop_servos)
  {
    cfg.names.push_back(joint + "/position");
  }

  // Sampler
  cfg.names.push_back(sampler_lift_l + "/position");
  cfg.names.push_back(sampler_lift_r + "/position");
  cfg.names.push_back(auger_spinner + "/velocity");
  cfg.names.push_back(auger_lift + "/position");

  return cfg; 
}

controller_interface::InterfaceConfiguration ScienceManual::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : state_joints_)
  {
    cfg.names.push_back(joint + "/position");
    cfg.names.push_back(joint + "/velocity");
  }

  return cfg;
}

controller_interface::CallbackReturn ScienceManual::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_controller_reference_msg(*(input_ref_.readFromRT)(), joystick_axes, joystick_buttons);
  if (command_interfaces_.size() != CMD_ITFS_COUNT) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "ScienceManual: expected %d command interfaces, got %zu",
      CMD_ITFS_COUNT, command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ScienceManual::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  stepper_pump_joints_.clear();
  dc_joints_.clear();
  servo_joints_.clear();
  rack_pinion_joints_.clear();
  state_joints_.clear();

  RCLCPP_INFO(get_node()->get_logger(), "Manual controller deactivated and released interfaces");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ScienceManual::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  auto current_ref = input_ref_.readFromRT();

  if (!(*current_ref)) {
    return controller_interface::return_type::OK;
  }

  double dt = period.seconds();
  auto msg = *current_ref;  // shared_ptr<sensor_msgs::msg::Joy>
  int stage_idx = static_cast<int>(current_mode_);  // corresponds to STAGE1..STAGE4

  // -- Pumps --
  // Stepper motors command (velocity-like value, but we feed as position target here)
  bool pump_button = (msg->buttons.size() > 2 && msg->buttons[2]);
  if (pump_button && !prev_pump_button_) {
      pump_toggle = !pump_toggle;
  }
  prev_pump_button_ = pump_button;
  stepper_cmd = pump_toggle * params_.velocity_limits_pumps[stage_idx];
  

  // -- Lift --
  // Rack & pinion
  // Use stage-dependent "speed" for rack & pinion motion (reuse stepper limits)
  double rack_speed = params_.velocity_limits_lift[stage_idx];
  double axis_left = (msg->axes.size() > 1) ? msg->axes[1] : 0.0;
  double axis_right = (msg->axes.size() > 1) ? msg->axes[3] : 0.0;

  // Integrate axes into positions (position += axis * speed * dt)

  double left_gain = 1.0;
  double right_gain = 1.0;
  // rack_left_position  += axis_left  * rack_speed * left_gain * dt;
  // rack_right_position -= axis_right * rack_speed * right_gain * dt;

  // ** TEMPORARY FOR SAR
  bool lift_button = (msg->buttons.size() > 11 && msg->buttons[12]);
  if (lift_button && !prev_lift_button) {
    lift_toggle = !lift_toggle;
  }
  prev_lift_button = lift_button;

  // Hardcoded values for top and bottom lift position
  rack_left_position  = lift_toggle ? 203.0 : 156.0;
  rack_right_position = lift_toggle ? 137.0 : 225.0;
  // **

  // -- Scoops --
  // Scoop Spinner (Right Trigger for Vel, Right Bumper for direction)
  double scoop_axis = (msg->axes.size() > 5) ? msg->axes[5] : 0.0;
  bool scoop_reverse = (msg->buttons.size() > 5 && msg->buttons[5]);
  double scoop_spinner_cmd = scoop_axis * params_.velocity_limits_scoop_spinner[stage_idx] * (scoop_reverse ? -1.0 : 1.0);

  // Scoop Servo
  // Servo A is square
  bool servo_scoop_a_button = (msg->buttons.size() > 1 && msg->buttons[3]);
  if (servo_scoop_a_button && !prev_servo_scoop_a_button_) {
    servo_scoop_a_toggle = !servo_scoop_a_toggle;
  }
  prev_servo_scoop_a_button_ = servo_scoop_a_button;
  scoop_servo_a_position = servo_scoop_a_toggle * params_.position_range_scoop_servo[1];

  // Servo B is X
  bool servo_scoop_b_button = (msg->buttons.size() > 0 && msg->buttons[0]);
  if (servo_scoop_b_button && !prev_servo_scoop_b_button_) {
    servo_scoop_b_toggle = !servo_scoop_b_toggle;
  }
  prev_servo_scoop_b_button_ = servo_scoop_b_button;
  scoop_servo_b_position = servo_scoop_b_toggle * params_.position_range_scoop_servo[1];

  // -- Sampler --
  // Sampler Lift
  double sampler_lift_speed = params_.velocity_limits_sampler_lift[stage_idx];
  double axis_sampler_lift = (msg->axes.size() > 3) ? msg->axes[3] : 0.0;
  sampler_lift_pos_l += sampler_lift_speed * axis_sampler_lift * dt;
  sampler_lift_pos_r -= sampler_lift_speed * axis_sampler_lift * dt;

  // Auger (Left Trigger for Vel, Left Bumper for direction)
  double auger_axis = (msg->axes.size() > 4) ? msg->axes[4] : 0.0;
  bool auger_reverse = (msg->buttons.size() > 4 && msg->buttons[4]);
  double auger_spinner_cmd = auger_axis * params_.velocity_limits_auger[stage_idx] * (auger_reverse ? -1.0 : 1.0);

  // Auger_lift
  // bool auger_lift_button = (msg->buttons.size() > 0 && msg->buttons[1]);
  // if (auger_lift_button && !prev_auger_lift_button_) {
  //   auger_lift_toggle = !auger_lift_toggle;
  // }
  // prev_auger_lift_button_ = auger_lift_button;
  // auger_lift_position = auger_lift_toggle ? params_.position_range_auger_lift[1] : params_.position_range_auger_lift[0];
  double auger_lift_axis = (msg->axes.size() > 0) ? msg->axes[0] : 0.0;

  // Use absolute value so direction doesn't matter
  double axis_mag = std::abs(auger_lift_axis);

  // Clamp just in case
  axis_mag = std::clamp(axis_mag, 0.0, 1.0);

  double min_pos = params_.position_range_auger_lift[0];
  double max_pos = params_.position_range_auger_lift[1];

  // Default = max_pos
  // More joystick deflection → move toward min_pos
  auger_lift_position = max_pos - axis_mag * (max_pos - min_pos);

  // Clamp Positions
  rack_left_position = std::clamp(rack_left_position,  0.0, 360.0);
  rack_right_position = std::clamp(rack_right_position, 0.0, 360.0);
  scoop_servo_a_position = std::clamp(scoop_servo_a_position, params_.position_range_scoop_servo[0], params_.position_range_scoop_servo[1]);
  scoop_servo_b_position = std::clamp(scoop_servo_b_position, params_.position_range_scoop_servo[0], params_.position_range_scoop_servo[1]);
  sampler_lift_pos_l = std::clamp(sampler_lift_pos_l, params_.position_range_sampler_lift[0], params_.position_range_sampler_lift[1]);
  sampler_lift_pos_r = std::clamp(sampler_lift_pos_r, params_.position_range_sampler_lift[0], params_.position_range_sampler_lift[1]);
  auger_lift_position = std::clamp(auger_lift_position, params_.position_range_auger_lift[0], params_.position_range_auger_lift[1]);
  
  // SET VALUES
  // Stepper motors (position)
  command_interfaces_[IDX_PUMP_A_VELOCITY].set_value(stepper_cmd * (M_PI / 180.0));
  command_interfaces_[IDX_PUMP_B_VELOCITY].set_value(stepper_cmd * (M_PI / 180.0));
  // DC motors (velocity)
  command_interfaces_[IDX_SAMPLER_LIFT_LEFT_VELOCITY].set_value(sampler_lift_pos_l * (M_PI / 180.0));
  command_interfaces_[IDX_SAMPLER_LIFT_RIGHT_VELOCITY].set_value(sampler_lift_pos_r * (M_PI / 180.0));
  command_interfaces_[IDX_SCOOP_SPINNER_VELOCITY].set_value(scoop_spinner_cmd);
  // Scoop servos
  command_interfaces_[IDX_SCOOP_A_POSITION].set_value(scoop_servo_a_position * (M_PI / 180.0));
  servo_scoop_b_counter++; // TESTING
  // if (servo_scoop_b_counter % 100 == 0) { // TESTING
  command_interfaces_[IDX_SCOOP_B_POSITION].set_value(scoop_servo_b_position * (M_PI / 180.0));
  // }
  // Auger & auger_lift
  command_interfaces_[IDX_AUGER_SPINNER_VELOCITY].set_value(auger_spinner_cmd);
  command_interfaces_[IDX_AUGER_LIFT_POSITION].set_value(auger_lift_position * (M_PI / 180.0));
  // Rack & pinion servos
  command_interfaces_[IDX_LEFT_LIFT_POSITION].set_value(rack_left_position * (M_PI / 180.0));
  command_interfaces_[IDX_RIGHT_LIFT_POSITION].set_value(rack_right_position * (M_PI / 180.0));

  std::ostringstream oss;
  oss << "\033[2J\033[H\n"
    << "========= Science Controller =========\n\n"

    << "  [PUMPS]\n"
    << "  triangle            : " << pump_button
    << " | pump_toggle        : " << pump_toggle
    << " | stepper_cmd        : " << std::fixed << std::setprecision(3) << stepper_cmd << "\n\n"

    << "  [LIFT]\n"
    << "  right_joystick_btn  : " << lift_button
    << " | lift_toggle        : " << lift_toggle << "\n"
    << "  rack_left_position  : " << rack_left_position << "\n"
    << "  rack_right_position : " << rack_right_position << "\n\n"

    << "  [SCOOPS]\n"
    << "  right_trigger_axis  : " << scoop_axis
    << " | right_bumper       : " << scoop_reverse
    << " | scoop_spinner_cmd  : " << scoop_spinner_cmd << "\n"
    << "  square              : " << servo_scoop_a_button
    << " | toggle             : " << servo_scoop_a_toggle
    << " | scoop_servo_a_pos  : " << scoop_servo_a_position << "\n"
    << "  x                   : " << servo_scoop_b_button
    << " | toggle             : " << servo_scoop_b_toggle
    << " | scoop_servo_b_pos  : " << scoop_servo_b_position << "\n\n"

    << "  [SAMPLER]\n"
    << "  right_joystick_ud   : " << axis_sampler_lift
    << " | sampler_lift_pos_l : " << sampler_lift_pos_l
    << " | sampler_lift_pos_r : " << sampler_lift_pos_r << "\n"
    << "  left_trigger_axis   : " << auger_axis
    << " | left_bumper        : " << auger_reverse
    << " | auger_spinner_cmd  : " << auger_spinner_cmd << "\n"
    << "  circle              : "
    << ((msg->buttons.size() > 1) ? msg->buttons[1] : 0)
    << " | auger_lift_cmd     : " << auger_lift_position << "\n";

  if(DEBUG_MODE == 1){
    RCLCPP_INFO(get_node()->get_logger(), "%s", oss.str().c_str());
  }

  // Basic state publish (still reusing existing signals)
  for (const auto & joint_name : joints_) {
    if (state_publisher_ && state_publisher_->trylock()) {
      state_publisher_->msg_.header.stamp = get_node()->get_clock()->now();
      state_publisher_->msg_.header.frame_id = joint_name;
      state_publisher_->msg_.set_point = stepper_cmd;
      state_publisher_->msg_.process_value = auger_spinner_cmd;
      state_publisher_->msg_.command = sampler_lift_pos_l;
      state_publisher_->unlockAndPublish();
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace science_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  science_controllers::ScienceManual, controller_interface::ControllerInterface)
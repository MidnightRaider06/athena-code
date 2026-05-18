#include "talon_ros2_control/talon_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace talon_ros2_control
{

TALONHardwareInterface::TALONHardwareInterface() = default;

void TALONHardwareInterface::logger_function()
{
  if (num_joints == 0) {
    return;
  }

  std::ostringstream oss;
  oss << "\033[2J\033[H \nTALON Logger"
      << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface
      << " | HWI Update Rate: " << update_rate
      << " | Logger Update Rate: " << logger_rate << "\n"
      << "Elapsed Time since first update: " << elapsed_time << "\n"
      << "\n--- Joint Specific ---";

  for (const auto & joint : TALONJoints_) {
    const char * control_mode = joint.control_level == integration_level_t::POSITION ? "POSITION" :
      (joint.control_level == integration_level_t::VELOCITY ? "VELOCITY" : "UNDEFINED");
    const char * joint_type = joint.joint_type == joint_type_t::PRISMATIC ? "PRISMATIC" : "REVOLUTE";

    oss << "\nJOINT: " << joint.name << "\n"
        << "Parameters: Node ID: 0x" << std::hex << std::uppercase << joint.node_id << std::dec << "\n"
        << "-- Commands --\n"
        << "Control Mode: " << control_mode << "\n"
        << "Joint Type: " << joint_type << "\n"
        << "Joint Command Position: " << joint.joint_command_position << "\n"
        << "Joint Command Velocity: " << joint.joint_command_velocity << "\n"
        << "Status Request: " << joint.motor_status_req << "\n"
        << "-- State --\n"
        << "Joint Position: " << joint.joint_state_position
        << " | Joint Velocity: " << joint.joint_state_velocity
        << " | Status: " << joint.motor_status << "\n";
  }

  RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "%s", oss.str().c_str());
}

hardware_interface::CallbackReturn TALONHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  can_interface = info_.hardware_parameters.at("can_interface");
  update_rate = std::stoi(info_.hardware_parameters.at("update_rate"));
  logger_rate = std::stoi(info_.hardware_parameters.at("logger_rate"));
  logger_state = std::stoi(info_.hardware_parameters.at("logger_state"));

  TALONJoints_.clear();
  for (const auto & joint : info_.joints) {
    std::vector<std::string> state_if_names;
    for (const auto & si : joint.state_interfaces) {
      state_if_names.push_back(si.name);
    }

    std::vector<std::string> command_if_names;
    for (const auto & ci : joint.command_interfaces) {
      command_if_names.push_back(ci.name);
    }

    std::unordered_map<std::string, std::string> params_map;
    for (const auto & p : joint.parameters) {
      params_map.emplace(p.first, p.second);
    }

    MotorConfig mc;
    if (joint.parameters.count("encoder_resolution")) {
      mc.encoder_resolution = std::stod(joint.parameters.at("encoder_resolution"));
    }
    if (joint.parameters.count("gear_ratio")) {
      mc.gear_ratio = std::abs(std::stod(joint.parameters.at("gear_ratio")));
    }
    if (joint.parameters.count("distance_per_rev")) {
      mc.distance_per_rev = std::stod(joint.parameters.at("distance_per_rev"));
    }
    if (joint.parameters.count("inverted")) {
      mc.inverted = (joint.parameters.at("inverted") == "true");
    }
    if (joint.parameters.count("sensor_phase")) {
      mc.sensor_phase = (joint.parameters.at("sensor_phase") == "true");
    }
    if (joint.parameters.count("kP")) {
      mc.kP = std::stod(joint.parameters.at("kP"));
    }
    if (joint.parameters.count("kI")) {
      mc.kI = std::stod(joint.parameters.at("kI"));
    }
    if (joint.parameters.count("kD")) {
      mc.kD = std::stod(joint.parameters.at("kD"));
    }
    if (joint.parameters.count("kF")) {
      mc.kF = std::stod(joint.parameters.at("kF"));
    }
    if (joint.parameters.count("config_timeout_ms")) {
      mc.config_timeout_ms = std::stoi(joint.parameters.at("config_timeout_ms"));
    }

    const joint_type_t joint_type = joint.parameters.at("joint_type") == "prismatic" ?
      joint_type_t::PRISMATIC : joint_type_t::REVOLUTE;

    TALONJoints_.push_back(TalonJoint{
      joint.name,
      std::stoi(joint.parameters.at("node_id")),
      joint_type,
      joint.parameters.count("max_disp") ? std::abs(std::stod(joint.parameters.at("max_disp"))) :
        std::numeric_limits<double>::quiet_NaN(),
      mc,
      nullptr,
      integration_level_t::POSITION,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      state_if_names,
      command_if_names,
      params_map
    });
  }

  num_joints = static_cast<int>(TALONJoints_.size());
  elapsed_update_time = 0.0;
  elapsed_time = 0.0;
  elapsed_logger_time = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TALONHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

std::vector<hardware_interface::StateInterface> TALONHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & joint : TALONJoints_) {
    for (const auto & iface : joint.state_interface_names) {
      double * value = nullptr;
      if (iface == hardware_interface::HW_IF_POSITION) {
        value = &joint.joint_state_position;
      } else if (iface == hardware_interface::HW_IF_VELOCITY) {
        value = &joint.joint_state_velocity;
      } else if (iface == "motor_temperature") {
        value = &joint.motor_temperature;
      } else if (iface == "torque_current") {
        value = &joint.motor_torque_current;
      } else if (iface == "status") {
        value = &joint.motor_status;
      }

      if (value != nullptr) {
        state_interfaces.emplace_back(joint.name, iface, value);
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TALONHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto & joint : TALONJoints_) {
    for (const auto & iface : joint.command_interface_names) {
      double * value = nullptr;
      if (iface == hardware_interface::HW_IF_POSITION) {
        value = &joint.joint_command_position;
      } else if (iface == hardware_interface::HW_IF_VELOCITY) {
        value = &joint.joint_command_velocity;
      } else if (iface == "status_request") {
        value = &joint.motor_status_req;
      }

      if (value != nullptr) {
        command_interfaces.emplace_back(joint.name, iface, value);
      }
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn TALONHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  for (auto & joint : TALONJoints_) {
    joint.motor = new TalonSRX(joint.node_id, can_interface);
    initMotor(joint.motor, joint.motor_config, can_interface);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TALONHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  for (auto & joint : TALONJoints_) {
    if (joint.motor != nullptr) {
      stopMotor(joint.motor, 50);
      delete joint.motor;
      joint.motor = nullptr;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

void TALONHardwareInterface::enable_system_thread()
{
  while (is_running.load()) {
    ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

hardware_interface::CallbackReturn TALONHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  is_running.store(true);
  worker = std::thread(&TALONHardwareInterface::enable_system_thread, this);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TALONHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (auto & joint : TALONJoints_) {
    if (joint.motor != nullptr) {
      stopMotor(joint.motor, 50);
    }
  }

  is_running.store(false);
  if (worker.joinable()) {
    worker.join();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TALONHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto & joint : TALONJoints_) {
    if (joint.motor == nullptr) {
      continue;
    }

    if (joint.joint_type == joint_type_t::PRISMATIC) {
      joint.joint_state_position = getPositionDistance(joint.motor, joint.motor_config);
      joint.joint_state_velocity = getLinearVelocity(joint.motor, joint.motor_config);
    } else {
      joint.joint_state_position = getPositionRadians(joint.motor, joint.motor_config);
      joint.joint_state_velocity = getAngularVelocity(joint.motor, joint.motor_config);
    }

    joint.motor_temperature = joint.motor->GetTemperature();
    joint.motor_torque_current = joint.motor->GetOutputCurrent();
    joint.motor_status = 1.0;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TALONHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  elapsed_update_time += period.seconds();
  elapsed_time += period.seconds();
  elapsed_logger_time += period.seconds();

  if (logger_rate > 0 && elapsed_logger_time > (1.0 / static_cast<double>(logger_rate))) {
    elapsed_logger_time = 0.0;
    if (logger_state == 1) {
      logger_function();
    }
  }

  for (auto & joint : TALONJoints_) {
    const double curr_status_req = joint.motor_status_req;
    if (curr_status_req < 0.0 && joint.prev_status_req >= 0.0) {
      joint.motor_status = 1.0;
    } else if (curr_status_req > 0.0) {
      joint.elapsed_status_request_time += period.seconds();
      if (joint.elapsed_status_request_time > (1.0 / curr_status_req)) {
        joint.elapsed_status_request_time = 0.0;
        joint.motor_status = 1.0;
      }
    }
    joint.prev_status_req = curr_status_req;
  }

  if (update_rate > 0 && elapsed_update_time > (1.0 / static_cast<double>(update_rate))) {
    elapsed_update_time = 0.0;
    for (auto & joint : TALONJoints_) {
      if (joint.motor == nullptr) {
        continue;
      }

      if (
        joint.control_level == integration_level_t::POSITION &&
        joint.joint_type == joint_type_t::PRISMATIC &&
        std::isfinite(joint.joint_command_position))
      {
        joint.joint_command_position = std::clamp(joint.joint_command_position, 0.0, joint.max_disp);
        setPositionFromDisplacement(joint.motor, joint.joint_command_position, 50, joint.motor_config);
      } else if (
        joint.control_level == integration_level_t::VELOCITY &&
        joint.joint_type == joint_type_t::PRISMATIC &&
        std::isfinite(joint.joint_command_velocity))
      {
        setVelocityFromLinearVelocity(joint.motor, joint.joint_command_velocity, 50, joint.motor_config);
      } else if (
        joint.control_level == integration_level_t::POSITION &&
        joint.joint_type == joint_type_t::REVOLUTE &&
        std::isfinite(joint.joint_command_position))
      {
        setPositionFromJointCommand(joint.motor, joint.joint_command_position, 50, joint.motor_config);
      } else if (
        joint.control_level == integration_level_t::VELOCITY &&
        joint.joint_type == joint_type_t::REVOLUTE &&
        std::isfinite(joint.joint_command_velocity))
      {
        setVelocityFromAngularVelocity(joint.motor, joint.joint_command_velocity, 50, joint.motor_config);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TALONHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  std::vector<integration_level_t> requested_modes(
    static_cast<size_t>(num_joints), integration_level_t::UNDEFINED);

  for (const auto & ifname : stop_interfaces) {
    for (size_t i = 0; i < TALONJoints_.size(); ++i) {
      const auto & joint = TALONJoints_[i];
      const std::string pos_if = joint.name + "/" + hardware_interface::HW_IF_POSITION;
      const std::string vel_if = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
      if (ifname == pos_if || ifname == vel_if || ifname.find(joint.name) != std::string::npos) {
        requested_modes[i] = integration_level_t::UNDEFINED;
      }
    }
  }

  for (const auto & ifname : start_interfaces) {
    for (size_t i = 0; i < TALONJoints_.size(); ++i) {
      const auto & joint = TALONJoints_[i];
      const std::string pos_if = joint.name + "/" + hardware_interface::HW_IF_POSITION;
      const std::string vel_if = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
      if (ifname == pos_if) {
        requested_modes[i] = integration_level_t::POSITION;
      } else if (ifname == vel_if) {
        requested_modes[i] = integration_level_t::VELOCITY;
      }
    }
  }

  for (size_t i = 0; i < TALONJoints_.size(); ++i) {
    auto & joint = TALONJoints_[i];
    if (requested_modes[i] == integration_level_t::UNDEFINED) {
      bool was_stopped = false;
      for (const auto & ifname : stop_interfaces) {
        if (ifname.find(joint.name) != std::string::npos) {
          was_stopped = true;
          break;
        }
      }
      if (was_stopped) {
        joint.control_level = integration_level_t::UNDEFINED;
        joint.joint_command_velocity = 0.0;
        joint.joint_command_position = joint.joint_state_position;
      }
      continue;
    }

    joint.control_level = requested_modes[i];
    if (joint.control_level == integration_level_t::VELOCITY) {
      joint.joint_command_velocity = 0.0;
    } else {
      joint.joint_command_position = joint.joint_state_position;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace talon_ros2_control

PLUGINLIB_EXPORT_CLASS(
  talon_ros2_control::TALONHardwareInterface, hardware_interface::SystemInterface)

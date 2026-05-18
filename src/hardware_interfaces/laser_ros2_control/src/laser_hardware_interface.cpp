#include "laser_ros2_control/laser_hardware_interface.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <sstream>

namespace laser_ros2_control
{

void LaserHardwareInterface::logger_function()
{
  if (LASERJoints_.empty()) {
    return;
  }

  const auto & joint = LASERJoints_.front();
  std::ostringstream oss;
  oss << "\033[2J\033[H \nLASER Logger"
      << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface_
      << " | HWI Update Rate: " << update_rate_
      << " | Logger Update Rate: " << logger_rate_ << "\n"
      << "Elapsed Time since first update: " << elapsed_time_ << "\n"
      << "\n--- Joint Specific ---\n"
      << "JOINT: " << joint.name << "\n"
      << "Parameters: CAN ID: 0x" << std::hex << std::uppercase << joint.can_id << std::dec << "\n"
      << "-- Commands --\n"
      << "Laser Command: " << joint.laser_command
      << " | Status Request: " << joint.status_request << "\n"
      << "-- State --\n"
      << "Laser State: " << joint.laser_state
      << " | Temperature: " << joint.temperature
      << " | Is Connected: " << joint.is_connected
      << " | Status: " << joint.status << "\n";

  RCLCPP_INFO(rclcpp::get_logger("LaserHardwareInterface"), "%s", oss.str().c_str());
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  can_interface_ = info_.hardware_parameters.count("can_interface") ?
    info_.hardware_parameters.at("can_interface") : "can0";
  update_rate_ = info_.hardware_parameters.count("update_rate") ?
    std::stoi(info_.hardware_parameters.at("update_rate")) : 0;
  logger_rate_ = info_.hardware_parameters.count("logger_rate") ?
    std::stoi(info_.hardware_parameters.at("logger_rate")) : 0;
  logger_state_ = info_.hardware_parameters.count("logger_state") ?
    std::stoi(info_.hardware_parameters.at("logger_state")) : 0;
  const uint32_t can_id = info_.hardware_parameters.count("can_id") ?
    static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0)) : 0x130;

  LASERJoints_.clear();
  LASERJoints_.push_back(LaserJoint{
    info_.gpios[0].name,
    can_id,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
  });

  can_connected_ = false;
  elapsed_time_ = 0.0;
  elapsed_logger_time_ = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!canBus_.open(
      can_interface_,
      std::bind(&LaserHardwareInterface::onCanMessage, this, std::placeholders::_1)))
  {
    can_connected_ = false;
  } else {
    can_connected_ = true;
  }
  auto & joint = LASERJoints_.front();
  joint.is_connected = can_connected_ ? 1.0 : 0.0;
  joint.status = joint.is_connected;
  return hardware_interface::CallbackReturn::SUCCESS;
}

void LaserHardwareInterface::onCanMessage(const CANLib::CanFrame & frame)
{
  auto & joint = LASERJoints_.front();
  if (frame.id == joint.can_id && frame.dlc > 1 && frame.data[0] == CMD_READ_TEMP) {
    joint.temperature = static_cast<double>(frame.data[1]);
    joint.status = 1.0;
  }
}

std::vector<hardware_interface::StateInterface> LaserHardwareInterface::export_state_interfaces()
{
  auto & joint = LASERJoints_.front();
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(joint.name, "laser_state", &joint.laser_state);
  state_interfaces.emplace_back(joint.name, "temperature", &joint.temperature);
  state_interfaces.emplace_back(joint.name, "is_connected", &joint.is_connected);
  state_interfaces.emplace_back(joint.name, "status", &joint.status);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LaserHardwareInterface::export_command_interfaces()
{
  auto & joint = LASERJoints_.front();
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(joint.name, "laser_command", &joint.laser_command);
  command_interfaces.emplace_back(joint.name, "status_request", &joint.status_request);
  return command_interfaces;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  auto & joint = LASERJoints_.front();
  if (can_connected_) {
    can_tx_frame_ = CANLib::CanFrame();
    can_tx_frame_.id = joint.can_id;
    can_tx_frame_.dlc = 1;
    can_tx_frame_.data[0] = CMD_LASER_OFF;
    canBus_.send(can_tx_frame_);
  }
  joint.laser_state = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  auto & joint = LASERJoints_.front();
  if (can_connected_) {
    can_tx_frame_ = CANLib::CanFrame();
    can_tx_frame_.id = joint.can_id;
    can_tx_frame_.dlc = 1;
    can_tx_frame_.data[0] = CMD_LASER_OFF;
    canBus_.send(can_tx_frame_);
    canBus_.close();
  }
  can_connected_ = false;
  joint.is_connected = 0.0;
  joint.status = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

hardware_interface::return_type LaserHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LaserHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  auto & joint = LASERJoints_.front();

  elapsed_time_ += period.seconds();
  elapsed_logger_time_ += period.seconds();
  if (logger_rate_ > 0 && elapsed_logger_time_ > (1.0 / static_cast<double>(logger_rate_))) {
    elapsed_logger_time_ = 0.0;
    if (logger_state_ == 1) {
      logger_function();
    }
  }

  const bool commanded_on = joint.laser_command > 0.5;
  const bool currently_on = joint.laser_state > 0.5;
  if (commanded_on != currently_on) {
    if (can_connected_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = joint.can_id;
      can_tx_frame_.dlc = 1;
      can_tx_frame_.data[0] = commanded_on ? CMD_LASER_ON : CMD_LASER_OFF;
      canBus_.send(can_tx_frame_);
    }
    joint.laser_state = commanded_on ? 1.0 : 0.0;
  }

  if (joint.status_request < 0.0 && joint.prev_status_request >= 0.0) {
    if (can_connected_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = joint.can_id;
      can_tx_frame_.dlc = 1;
      can_tx_frame_.data[0] = CMD_READ_TEMP;
      canBus_.send(can_tx_frame_);
    }
  } else if (joint.status_request > 0.0) {
    joint.elapsed_status_request_time += period.seconds();
    if (joint.elapsed_status_request_time > (1.0 / joint.status_request)) {
      joint.elapsed_status_request_time = 0.0;
      if (can_connected_) {
        can_tx_frame_ = CANLib::CanFrame();
        can_tx_frame_.id = joint.can_id;
        can_tx_frame_.dlc = 1;
        can_tx_frame_.data[0] = CMD_READ_TEMP;
        canBus_.send(can_tx_frame_);
      }
    }
  }
  joint.prev_status_request = joint.status_request;

  return hardware_interface::return_type::OK;
}

}  // namespace laser_ros2_control

PLUGINLIB_EXPORT_CLASS(
  laser_ros2_control::LaserHardwareInterface, hardware_interface::SystemInterface)

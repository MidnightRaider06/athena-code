#ifndef SMC_HARDWARE_INTERACE_HPP_
#define SMC_HARDWARE_INTERACE_HPP_

#include <netinet/in.h>

#include <array>
#include <cstdint>
#include <limits>
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

namespace smc_ros2_control
{

class SMCHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SMCHardwareInterface)

  // -- Lifecycle Functions --
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

  enum class integration_level_t : std::uint8_t
  {
    UNDEFINED = 0,
    POSITION = 1,
    VELOCITY = 2,
  };

  enum class SMCMotorStatus : std::uint8_t
  {
    OK = 0x00,
    LOW_VOLTAGE = 0x01,
    OVER_TEMPERATURE = 0x08,
  };

  struct SMCJoint
  {
    std::string name;
    uint32_t node_id;
    int gear_ratio;
    int orientation;
    uint16_t operating_velocity;
    integration_level_t control_level;

    double joint_state_position;
    double joint_state_velocity;
    double motor_temperature;
    double motor_torque_current;
    double motor_status;
    double current_Kp;
    double current_Ki;
    double current_Kd;
    double speed_Kp;
    double speed_Ki;
    double speed_Kd;
    double position_Kp;
    double position_Ki;
    double position_Kd;
    double acceleration;
    double max_torque;
    double max_speed;
    double max_angle;
    double current_ramp;
    double speed_ramp;

    double joint_command_position;
    double joint_command_velocity;
    double motor_status_req;
    double motor_maintenance_req;
    double maintenance_frame_high;
    double maintenance_frame_low;
    double maintenance_frame;
    double maintenance_data_count;
    std::vector<uint8_t> decoded_maintenance_frame;

    double prev_status_req;
    double prev_maintenance_req;
    double elapsed_status_request_time;
    double elapsed_maintenance_request_time;
    double motor_velocity;
    double motor_position;
    double encoder_position;
    double prev_joint_command_position;
    double prev_joint_command_velocity;

    std::vector<std::string> state_interface_names;
    std::vector<std::string> command_interface_names;
    std::unordered_map<std::string, std::string> parameters;
  };

  struct DecodedCommand
  {
    uint8_t command_id;
    std::vector<uint8_t> u8_data;
    std::vector<int16_t> i16_data;
    std::vector<int32_t> i32_data;
  };

  void send_command(int can_id, int cmd_id);
  void on_can_message(const CANLib::CanFrame & frame);
  void logger_function();
  double calculate_joint_position_from_motor_position(double motor_position, int gear_ratio);
  double calculate_joint_velocity_from_motor_velocity(double motor_velocity, int gear_ratio);
  int32_t calculate_motor_position_from_desired_joint_position(double joint_position, int gear_ratio);
  int32_t calculate_motor_velocity_from_desired_joint_velocity(double joint_velocity, int gear_ratio);
  bool interpret_settings_parameters(SMCJoint & joint, const std::array<uint8_t, 8> & data);
  void format_control_command(CANLib::CanFrame & frame, SMCJoint & joint);
  bool format_status_command(CANLib::CanFrame & frame, uint8_t command_id);
  bool format_maintenance_command(CANLib::CanFrame & frame, const DecodedCommand & decoded_cmd);

private:
  // Hardware Interface Parameters
  int update_rate;
  double elapsed_update_time; // Time since last hardware interface update
  double elapsed_time; // Time since first hardware interface update
  double elapsed_logger_time; // Time since last logger update
  int logger_rate; // Logger update rate
  int logger_state; // Logger on/off state

  int num_joints;
  int state_iterator;

  CANLib::SocketCanBus canBus;
  CANLib::CanFrame can_tx_frame_;
  CANLib::CanFrame can_rx_frame_;
  std::string can_interface;

  std::vector<SMCJoint> SMCJoints_;

  enum class MaintenanceCommands : uint8_t
  {
    WRITE_ACCELERATION_CMD = 0x34,
    READ_SETTINGS_CMD = 0x40,
    WRITE_SETTINGS_TO_RAM_CMD = 0x42,
    WRITE_SETTINGS_TO_ROM_CMD = 0x44,
    MOTOR_SHUTDOWN_CMD = 0x80,
    MOTOR_STOP_CMD = 0x81,
    MOTOR_RUNNING_CMD = 0x88,
    CLEAR_MOTOR_ANGLE_CMD = 0x95,
    CLEAR_ERROR_CMD = 0x9B,
  };

  enum class ControlCommands : uint8_t
  {
    TORQUE_CONTROL_CMD = 0xA1,
    SPEED_CONTROL_CMD = 0xA2,
    ABSOLUTE_POS_WITH_VEL_CONTROL_CMD = 0xA4,
  };

  enum class StatusCommands : uint8_t
  {
    READ_ACCELERATION_CMD = 0x33,
    READ_ENCODER_CMD = 0x90,
    READ_ABS_ANGLE_CMD = 0x92,
    MOTOR_STATUS_1_CMD = 0x9A,
    MOTOR_STATUS_2_CMD = 0x9C,
  };

  enum class ParamID : uint8_t
  {
    ANGLE_PID = 0x96,
    SPEED_PID = 0x97,
    CURRENT_PID = 0x98,
    MAX_TORQUE = 0x99,
    MAX_SPEED = 0x9A,
    MAX_ANGLE_A = 0x9B,
    MAX_ANGLE_B = 0x9C,
    CURRENT_RAMP = 0x9D,
    SPEED_RAMP = 0x9E,
  };

  static constexpr std::array<StatusCommands, 6> kStatusCommands = {
    StatusCommands::READ_ACCELERATION_CMD,
    StatusCommands::READ_ENCODER_CMD,
    StatusCommands::READ_ABS_ANGLE_CMD,
    StatusCommands::MOTOR_STATUS_1_CMD,
    StatusCommands::MOTOR_STATUS_2_CMD,
  };

  inline DecodedCommand unpack_command_full(int32_t counts_in, int64_t payload_in)
  {
    const uint32_t counts = static_cast<uint32_t>(counts_in);
    const uint64_t payload = static_cast<uint64_t>(payload_in);

    const uint8_t u8_count = static_cast<uint8_t>((counts >> 16) & 0xFF);
    const uint8_t i16_count = static_cast<uint8_t>((counts >> 8) & 0xFF);
    const uint8_t i32_count = static_cast<uint8_t>(counts & 0xFF);

    int bit_offset = 64;
    auto pop_bits = [&](int bits) -> uint64_t {
      bit_offset -= bits;
      const uint64_t mask = (bits == 64) ? std::numeric_limits<uint64_t>::max() : ((1ULL << bits) - 1ULL);
      return (payload >> bit_offset) & mask;
    };

    DecodedCommand result{};
    result.command_id = static_cast<uint8_t>(pop_bits(8));
    for (uint8_t i = 0; i < u8_count; ++i) {
      result.u8_data.push_back(static_cast<uint8_t>(pop_bits(8)));
    }
    for (uint8_t i = 0; i < i16_count; ++i) {
      result.i16_data.push_back(static_cast<int16_t>(static_cast<uint16_t>(pop_bits(16))));
    }
    for (uint8_t i = 0; i < i32_count; ++i) {
      result.i32_data.push_back(static_cast<int32_t>(static_cast<uint32_t>(pop_bits(32))));
    }
    return result;
  }

  template<typename JointT>
  inline void pack_decoded_maintenance_frame(
    JointT & joint, const DecodedCommand & decoded_maintenance_cmd)
  {
    joint.decoded_maintenance_frame.clear();
    joint.decoded_maintenance_frame.push_back(decoded_maintenance_cmd.command_id);
    joint.decoded_maintenance_frame.insert(
      joint.decoded_maintenance_frame.end(),
      decoded_maintenance_cmd.u8_data.begin(),
      decoded_maintenance_cmd.u8_data.end());

    const auto * i16_ptr = reinterpret_cast<const uint8_t *>(decoded_maintenance_cmd.i16_data.data());
    joint.decoded_maintenance_frame.insert(
      joint.decoded_maintenance_frame.end(),
      i16_ptr,
      i16_ptr + decoded_maintenance_cmd.i16_data.size() * sizeof(int16_t));

    const auto * i32_ptr = reinterpret_cast<const uint8_t *>(decoded_maintenance_cmd.i32_data.data());
    joint.decoded_maintenance_frame.insert(
      joint.decoded_maintenance_frame.end(),
      i32_ptr,
      i32_ptr + decoded_maintenance_cmd.i32_data.size() * sizeof(int32_t));
  }
};
}  // namespace smc_ros2_control
#endif  // SMC_HARDWARE_INTERACE_HPP_

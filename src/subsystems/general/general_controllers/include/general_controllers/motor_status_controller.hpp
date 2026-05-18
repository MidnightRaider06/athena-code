// Copyright (c) 2025, UMDLoop
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GENERAL_CONTROLLERS__motor_status_controller_HPP_
#define GENERAL_CONTROLLERS__motor_status_controller_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "msgs/msg/joint_status.hpp"
#include "msgs/msg/system_info.hpp"
#include "msgs/srv/maintenance_req.hpp"
#include "msgs/srv/status_req.hpp"
#include "general_controllers/visibility_control.h"
#include <general_controllers/motor_status_controller_parameters.hpp>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace general_controllers
{

class MotorStatusController : public controller_interface::ControllerInterface
{
public:
  GENERAL_CONTROLLERS__VISIBILITY_PUBLIC
  MotorStatusController();

  GENERAL_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  GENERAL_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  GENERAL_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  GENERAL_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  GENERAL_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  GENERAL_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  GENERAL_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  GENERAL_CONTROLLERS__VISIBILITY_PUBLIC
    void logger_function();

protected:
  std::shared_ptr<motor_status_controller::ParamListener> param_listener_;
  motor_status_controller::Params params_;
  
  // Publisher for motor status information
  rclcpp::Publisher<msgs::msg::SystemInfo>::SharedPtr motor_status_publisher_;

  // Services for command interfaces
  rclcpp::Service<msgs::srv::StatusReq>::SharedPtr status_request_service_;
  rclcpp::Service<msgs::srv::MaintenanceReq>::SharedPtr maintenance_request_service_;
  std::string status_req_resource_name;
  int status_request_rate;
  std::string maintenance_req_joint_name;
  int maintenance_request_rate;
  double maintenance_frame_high;
  double maintenance_frame_low;
  double maintenance_data_count;
  bool status_one_shot_sent;
  bool maintenance_one_shot_sent;

  // Map from "joint/interface" to index in state_interfaces_
  std::unordered_map<std::string, size_t> state_interface_map_;

  // Map from "joint/interface" to index in command_interfaces_
  std::unordered_map<std::string, size_t> command_interface_map_;

  // Publish rate limiting
  rclcpp::Duration publish_period_{0, 0};
  rclcpp::Time last_publish_time_{0, 0, RCL_CLOCK_UNINITIALIZED};
  rclcpp::Time status_one_shot_time{0, 0, RCL_CLOCK_UNINITIALIZED};
  rclcpp::Time maintenance_one_shot_time{0, 0, RCL_CLOCK_UNINITIALIZED};
  rclcpp::Duration one_shot_delay{0, 100000000};

    std::vector<std::string> available_state_interfaces;
    std::vector<std::string> available_command_interfaces;

  struct PackedCommand
  {
    int32_t counts; // 24 bits for count of each data type (u8, i16, i32)
    int64_t payload; // 8 bits for command_id, followed by packed data (u8, i16, i32)
  };

  inline PackedCommand pack_command_full(
      uint8_t command_id,
      const std::vector<uint8_t>& u8_data,
      const std::vector<int16_t>& i16_data,
      const std::vector<int32_t>& i32_data)
  {
    size_t total_bits = 8 + 8 * u8_data.size() + 16 * i16_data.size() + 32 * i32_data.size();
    if (total_bits > 64) {
        RCLCPP_ERROR(rclcpp::get_logger("MotorStatusController"), "Payload exceeds 64 bits: total_bits=%zu", total_bits);
        return PackedCommand{0, 0};
    }

    // PACK COUNTS (only 24 bits needed: 8 bits each for u8/i16/i32 counts)
    uint32_t counts = 0;
    int count_offset = 24;
    auto push_count = [&](uint32_t value, int bits) {
        count_offset -= bits;
        counts |= (value & ((1U << bits) - 1)) << count_offset;
    };
    push_count(static_cast<uint32_t>(u8_data.size()),  8);
    push_count(static_cast<uint32_t>(i16_data.size()), 8);
    push_count(static_cast<uint32_t>(i32_data.size()), 8);

    // PACK PAYLOAD
    uint64_t payload = 0;
    int payload_offset = 64;
    auto push_payload = [&](uint64_t value, int bits) {
        payload_offset -= bits;
        payload |= (value & ((1ULL << bits) - 1)) << payload_offset;
    };
    push_payload(command_id, 8);
    for (auto v : u8_data)  push_payload(v, 8);
    for (auto v : i16_data) push_payload(static_cast<uint16_t>(v), 16);
    for (auto v : i32_data) push_payload(static_cast<uint32_t>(v), 32);

    return PackedCommand{
        static_cast<int32_t>(counts),
        static_cast<int64_t>(payload)
    };
  }
  
  enum class MotorStatus : uint8_t {
    UNKNOWN = 0,
    IDLE = 1,
    ACTIVE = 2,
    WARNING = 3,
    ERROR = 4,
    DISABLED = 5
  };
};

}  // namespace general_controllers

#endif  // GENERAL_CONTROLLERS__motor_status_controller_HPP_

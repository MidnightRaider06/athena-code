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

#include "general_controllers/motor_status_controller.hpp"

#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

#include "pluginlib/class_list_macros.hpp"

#define DEBUG_MODE 0

namespace general_controllers
{

MotorStatusController::MotorStatusController() {}

void MotorStatusController::logger_function()
{
  std::string log_msg = "\033[2J\033[H \nMotor Status Controller Logger";
  
  // HWI Specific
  std::ostringstream oss;

  // for(const auto & joint : params_.joints) {
  //   if (available_state_interfaces.empty()) {
  //     RCLCPP_INFO(
  //       get_node()->get_logger(),
  //       "Joint '%s' has no motor status state interfaces available.",
  //       joint.c_str());
  //     continue;
  //   }

  //   std::ostringstream interfaces_stream;
  //   for (size_t i = 0; i < available_state_interfaces.size(); ++i) {
  //     if (i > 0) {
  //       interfaces_stream << ", ";
  //     }
  //     interfaces_stream << available_state_interfaces[i];
  //   }

  //   RCLCPP_INFO(
  //     get_node()->get_logger(),
  //     "Joint '%s' will publish available motor status interfaces: %s",
  //     joint.c_str(),
  //     interfaces_stream.str().c_str());

  //   if (available_command_interfaces.empty()) {
  //     RCLCPP_INFO(
  //       get_node()->get_logger(),
  //       "Joint '%s' has no motor status command interfaces available.",
  //       joint.c_str());
  //     continue;
  //   }

  //   interfaces_stream.str("");
  //   for (size_t i = 0; i < available_command_interfaces.size(); ++i) {
  //     if (i > 0) {
  //       interfaces_stream << ", ";
  //     }
  //     interfaces_stream << available_command_interfaces[i];
  //   }

  //   RCLCPP_INFO(
  //     get_node()->get_logger(),
  //     "Joint '%s' will use available motor status command interfaces: %s",
  //     joint.c_str(),
  //     interfaces_stream.str().c_str());
  // }
  
  log_msg += oss.str();
  RCLCPP_INFO(get_node()->get_logger(), log_msg.c_str());
}

controller_interface::CallbackReturn MotorStatusController::on_init()
{
  try {
    param_listener_ = std::make_shared<motor_status_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  status_req_resource_name = "";
  status_request_rate = 0;
  maintenance_req_joint_name = "";
  maintenance_request_rate = 0;
  status_one_shot_sent = false;
  maintenance_one_shot_sent = false;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MotorStatusController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & [joint_name, joint_cfg] : params_.command_interfaces.joints_map){
    for(const auto & interface_name : joint_cfg.interfaces){
      command_interfaces_config.names.push_back(joint_name + "/" + interface_name);
    }
  }
  for (const auto & [gpio_name, gpio_cfg] : params_.command_interfaces.gpios_map){
    for(const auto & interface_name : gpio_cfg.interfaces){
      command_interfaces_config.names.push_back(gpio_name + "/" + interface_name);
    }
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
MotorStatusController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & [joint_name, joint_cfg] : params_.state_interfaces.joints_map){
    for(const auto & interface_name : joint_cfg.interfaces){
      state_interfaces_config.names.push_back(joint_name + "/" + interface_name);
    }
  }
  for (const auto & [gpio_name, gpio_cfg] : params_.state_interfaces.gpios_map){
    for(const auto & interface_name : gpio_cfg.interfaces){
      state_interfaces_config.names.push_back(gpio_name + "/" + interface_name);
    }
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn MotorStatusController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (params_.joints.empty() && params_.gpios.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints or gpios specified for motor_status_controller.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // if (params_.state_interfaces.joints_map.empty()) {
  //   RCLCPP_ERROR(get_node()->get_logger(), "No interfaces specified for motor_status_controller.");
  //   return controller_interface::CallbackReturn::ERROR;
  // }

  // Set up publish rate
  if (params_.publish_rate > 0.0) {
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / params_.publish_rate);
  } else {
    publish_period_ = rclcpp::Duration(0, 0);
  }

  motor_status_publisher_ = get_node()->create_publisher<msgs::msg::SystemInfo>(
    "~/motor_status", rclcpp::SystemDefaultsQoS());

  status_request_service_ = get_node()->create_service<msgs::srv::StatusReq>(
    "~/status_request",
    [this](
      const std::shared_ptr<msgs::srv::StatusReq::Request> request,
      std::shared_ptr<msgs::srv::StatusReq::Response> response)
    {
      if (request->joint_name.empty()) {
        response->success = false;
        response->message = "Joint/GPIO name cannot be empty";
        RCLCPP_WARN(get_node()->get_logger(), "%s", response->message.c_str());
        return;
      }

      status_req_resource_name = request->joint_name;
      status_request_rate = request->request_rate;

      std::string msg =
        "Received status_request for resource " + status_req_resource_name +
        " at request_rate: " + std::to_string(status_request_rate);
      response->success = true;
      response->message = msg;
      RCLCPP_INFO(get_node()->get_logger(), "%s", msg.c_str());
    });

  maintenance_request_service_ = get_node()->create_service<msgs::srv::MaintenanceReq>(
    "~/maintenance_request",
    [this](
      const std::shared_ptr<msgs::srv::MaintenanceReq::Request> request,
      std::shared_ptr<msgs::srv::MaintenanceReq::Response> response)
    {
      if (request->joint_name.empty()) {
        response->success = false;
        response->message = "Joint name cannot be empty";
        RCLCPP_WARN(get_node()->get_logger(), "%s", response->message.c_str());
        return;
      }

      maintenance_req_joint_name = request->joint_name;
      maintenance_request_rate = request->request_rate;

      auto packed = pack_command_full(
                    request->command_id,
                    request->u8_data,
                    request->i16_data,
                    request->i32_data);

      auto payload_to_doubles = [](int64_t value) -> std::pair<double, double>
      {
          uint64_t u = static_cast<uint64_t>(value);
          double high = static_cast<double>(u >> 32);
          double low  = static_cast<double>(u & 0xFFFFFFFF);
          return {high, low};
      };

      auto [high, low] = payload_to_doubles(packed.payload);

      maintenance_frame_high = high;
      maintenance_frame_low  = low;
      maintenance_data_count = static_cast<double>(packed.counts);

      std::ostringstream oss;

      oss << "Received maintenance_request for joint "
          << maintenance_req_joint_name
          << " with frame: 0x"
          << std::hex << std::uppercase
          << std::setfill('0') << std::setw(8) << static_cast<uint64_t>(maintenance_frame_high)
          << std::setfill('0') << std::setw(8) << static_cast<uint64_t>(maintenance_frame_low)
          << std::dec << std::nouppercase
          << " and data count(s): "
          << maintenance_data_count
          << " at request_rate: "
          << maintenance_request_rate;

      std::string msg = oss.str();
      response->success = true;
      response->message = msg;
      RCLCPP_INFO(get_node()->get_logger(), "%s", msg.c_str());
    });

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured motor_status_controller for %zu joints and %zu gpios at publish rate: %.1f Hz",
    params_.joints.size(), params_.gpios.size(), params_.publish_rate);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotorStatusController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Build lookup map from "joint/interface" -> state_interfaces_ index
  state_interface_map_.clear();
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    state_interface_map_[state_interfaces_[i].get_prefix_name() + "/" +
                         state_interfaces_[i].get_interface_name()] = i;
  }

  // for (const auto & joint : params_.joints) {
  //   for (const auto & iface : params_.state_interfaces) {
  //     const std::string key = joint + "/" + iface;
  //     if (state_interface_map_.find(key) != state_interface_map_.end()) {
  //       available_state_interfaces.push_back(iface);
  //     }
  //   }
  // }

  // Build lookup map from "joint/interface" -> command_interfaces_ index
  command_interface_map_.clear();
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interface_map_[command_interfaces_[i].get_prefix_name() + "/" +
                          command_interfaces_[i].get_interface_name()] = i;
  }

  // for (const auto & joint : params_.joints) {
  //   for (const auto & iface : params_.command_interfaces) {
  //     const std::string key = joint + "/" + iface;
  //     if (command_interface_map_.find(key) != command_interface_map_.end()) {
  //       available_command_interfaces.push_back(iface);
  //     }
  //   }
  // }

  last_publish_time_ = get_node()->now();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotorStatusController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  state_interface_map_.clear();
  command_interface_map_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MotorStatusController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Rate limit publishing
  const bool periodic_publish_due =
    (publish_period_.seconds() <= 0.0 || (time - last_publish_time_) >= publish_period_);

  if (!periodic_publish_due) {
    return controller_interface::return_type::OK;
  }
  last_publish_time_ = time;

  msgs::msg::SystemInfo system_info_msg;
  system_info_msg.header.stamp = time;

  // Command Interfaces
  auto update_command_interfaces =
    [this, &time](const auto & interface_map, const bool allow_maintenance)
    {
      for (const auto & [resource_name, resource_cfg] : interface_map) {
        for (const auto & interface_name : resource_cfg.interfaces) {
          std::string key = resource_name + "/" + interface_name;
          auto it = command_interface_map_.find(key);
          if (it == command_interface_map_.end()) {
            continue;
          }

          if (allow_maintenance && interface_name == "maintenance_frame_high" &&
            resource_name == maintenance_req_joint_name)
          {
            command_interfaces_[it->second].set_value(maintenance_frame_high);
          }

          if (allow_maintenance && interface_name == "maintenance_frame_low" &&
            resource_name == maintenance_req_joint_name)
          {
            command_interfaces_[it->second].set_value(maintenance_frame_low);
          }

          if (allow_maintenance && interface_name == "maintenance_data_count" &&
            resource_name == maintenance_req_joint_name)
          {
            command_interfaces_[it->second].set_value(maintenance_data_count);
          }

          if (interface_name == "status_request" && resource_name == status_req_resource_name) {
            if (status_request_rate < 0 && status_one_shot_sent == false) {
              command_interfaces_[it->second].set_value(status_request_rate);
              RCLCPP_WARN(get_node()->get_logger(), "Status one shot sent: ");
              status_one_shot_sent = true;
              status_one_shot_time = time;
            }
            else if (status_request_rate < 0 && status_one_shot_sent == true &&
              (time - status_one_shot_time) >= one_shot_delay)
            {
              status_request_rate = 0;
              command_interfaces_[it->second].set_value(status_request_rate);
              status_one_shot_sent = false;
              RCLCPP_WARN(get_node()->get_logger(), "Status one shot reset.");
            }
            else if (status_request_rate >= 0) {
              command_interfaces_[it->second].set_value(status_request_rate);
            }
          }

          if (allow_maintenance && interface_name == "maintenance_request" &&
            resource_name == maintenance_req_joint_name)
          {
            if (maintenance_request_rate < 0 && maintenance_one_shot_sent == false) {
              command_interfaces_[it->second].set_value(maintenance_request_rate);
              RCLCPP_WARN(get_node()->get_logger(), "Maintenance one shot sent: ");
              maintenance_one_shot_sent = true;
              maintenance_one_shot_time = time;
            }
            else if (maintenance_request_rate < 0 && maintenance_one_shot_sent == true &&
              (time - maintenance_one_shot_time) >= one_shot_delay)
            {
              maintenance_request_rate = 0;
              command_interfaces_[it->second].set_value(maintenance_request_rate);
              maintenance_one_shot_sent = false;
              RCLCPP_WARN(get_node()->get_logger(), "Maintenance one shot reset.");
            }
            else if (maintenance_request_rate >= 0) {
              command_interfaces_[it->second].set_value(maintenance_request_rate);
            }
          }
        }
      }
    };

  update_command_interfaces(params_.command_interfaces.joints_map, true);
  update_command_interfaces(params_.command_interfaces.gpios_map, false);

  auto append_state_status =
    [this, &system_info_msg](const auto & interface_map)
    {
      for (const auto & [resource_name, resource_cfg] : interface_map) {
        msgs::msg::JointStatus status;
        status.joint_name = resource_name;
        status.temperature = std::numeric_limits<int8_t>::quiet_NaN();
        status.torque_current = std::numeric_limits<double>::quiet_NaN();
        status.motor_status = std::numeric_limits<int8_t>::quiet_NaN();

        for (const auto & interface_name : resource_cfg.interfaces) {
          std::string key = resource_name + "/" + interface_name;
          auto it = state_interface_map_.find(key);

          if (it == state_interface_map_.end()) {
            continue;
          }

          double value = state_interfaces_[it->second].get_value();

          if (interface_name == "motor_temperature") {
            status.temperature = static_cast<int8_t>(value);
          } else if (interface_name == "torque_current") {
            status.torque_current = value;
          } else if (interface_name == "status") {
            status.motor_status = static_cast<int8_t>(value);
            if (value > sizeof(MotorStatus)) {
              RCLCPP_WARN(get_node()->get_logger(), "Invalid motor status value");
            }
          }
        }

        system_info_msg.joints.push_back(status);
      }
    };

  append_state_status(params_.state_interfaces.joints_map);
  append_state_status(params_.state_interfaces.gpios_map);

  motor_status_publisher_->publish(system_info_msg);

  return controller_interface::return_type::OK;
}

}  // namespace general_controllers

PLUGINLIB_EXPORT_CLASS(
  general_controllers::MotorStatusController,
  controller_interface::ControllerInterface)

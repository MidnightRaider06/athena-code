// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

//
// Authors: Denis Stogl
//

#include "rmd_ros2_control/rmd_hardware_interface.hpp"

#include <netdb.h>
#include <sys/socket.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <cstring>
#include <sstream>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <cmath>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define DEBUG_MODE 0 // 0 for off 1 for on

using std::placeholders::_1;

namespace rmd_ros2_control
{
hardware_interface::CallbackReturn RMDHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info) // Info stores all parameters in xacro file
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  /*
  IF YOU WANT TO USE PARAMETERS FROM ROS2_CONTROL XACRO, DO THAT HERE!!!
  */
  
  // This stores the can ids for each joint aka motor
  for (auto& joint : info_.joints) {
    joint_node_write_ids.push_back(std::stoi(joint.parameters.at("node_write_id")));
    joint_node_read_ids.push_back(std::stoi(joint.parameters.at("node_read_id")));
    joint_gear_ratios.push_back(std::stoi(joint.parameters.at("gear_ratio")));
    initial_position_.push_back(std::stof(joint.state_interfaces[0].initial_value));
    joint_orientation.push_back(std::stoi(joint.parameters.at("joint_orientation")));
  }

  num_joints = static_cast<int>(info_.joints.size());

  for (size_t i = 0; i < initial_position_.size(); ++i) {
    RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Joint %zu initial position in on_init: %f", i, initial_position_[i]);
  }
  
  // Initializes command and state interface values
  joint_state_position_.assign(num_joints, 0.0);
  joint_state_velocity_.assign(num_joints, 0.0);

  // joint_command_position_.assign(num_joints, 0);
  joint_command_position_.assign(num_joints, std::numeric_limits<double>::quiet_NaN());

  // joint_command_position_ = initial_position_;
  joint_command_velocity_.assign(num_joints, 0.0);

  for (size_t i = 0; i < initial_position_.size(); ++i) {
    RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Joint %zu command vel in on_init: %f", i, joint_command_velocity_[i]);
  }

  encoder_position = 0.0;
  motor_position = 0.0;
  motor_velocity = 0.0;

  joint_initialization_.assign(num_joints, false);

  control_level_.resize(num_joints, integration_level_t::POSITION);

  current_iteration = 0;

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn RMDHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  // Reuse cleanup logic which shuts down the motor and then deinitializes shared pointers.
  // Need this in case on_cleanup never gets called
  return this->on_cleanup(previous_state);
}


std::vector<hardware_interface::StateInterface> RMDHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Each RMD motor corresponds to a different joint.
  for(int i = 0; i < num_joints; i++){   
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_state_position_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_state_velocity_[i]));
  }
  
  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface>
RMDHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for(int i = 0; i < num_joints; i++){
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_command_position_[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_command_velocity_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RMDHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: Find a way to publish from the lifecyle node itself rather than create our own node within it
  // Node to publish to umdloop_can_node
  node_ = rclcpp::Node::make_shared("rmd_hardware_node");
  rmd_can_publisher_ = node_->create_publisher<msgs::msg::CANA>("can_tx", rclcpp::QoS(10));

  // Lambda function that takes the message as a shared pointer, dereferences it, 
  // and stores it in received_joint_data_ to be used
  rmd_can_subscriber_ = node_->create_subscription<msgs::msg::CANA>(
      "can_rx", 
      rclcpp::QoS(50).reliable(), 
      [this](const msgs::msg::CANA::SharedPtr received_message) 
      {
        received_joint_data_ = *received_message;
      }
  );

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RMDHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // If cleanup occurs before shutdown, this is the last opportunity to shutdown motor since pointers must be deleted here
  for(int i = 0; i < num_joints; i++){
    auto joint_tx = msgs::msg::CANA();
    joint_tx.id = joint_node_write_ids[i];
        
    // Motor Shutdown Command
    joint_tx.data = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    rmd_can_publisher_->publish(joint_tx);

    // Brake Lock Command (don't think this works)
    joint_tx.data = {0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    rmd_can_publisher_->publish(joint_tx);
  }

  // Reset shared pointers which essentially deletes it
  rmd_can_publisher_.reset();
  rmd_can_subscriber_.reset();
  node_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RMDHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Activating ...please wait...");

  for(int i = 0; i < num_joints; i++){
    auto joint_tx = msgs::msg::CANA();
    joint_tx.id = joint_node_write_ids[i];
        
    // Brake Release command (pretty sure brakes don't work)
    joint_tx.data = {0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    rmd_can_publisher_->publish(joint_tx);
  }

  for (size_t i = 0; i < initial_position_.size(); ++i) {
    RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Joint %zu initial position in on_activate: %f", i, initial_position_[i]);
  }
  
  // Initialize command positions and velocities to zero to prevent initial movement
  // Controllers will override these once they start
  joint_command_position_.assign(num_joints, 0.0);
  joint_command_velocity_.assign(num_joints, 0.0);
  
  for (size_t i = 0; i < joint_command_position_.size(); ++i) {
    RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Joint %zu command position initialized to: %f", i, joint_command_position_[i]);
    RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Joint %zu command velocity initialized to: %f", i, joint_command_velocity_[i]);
  }

  RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn RMDHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Deactivating ...please wait...");

  for(int i = 0; i < num_joints; i++){
    auto joint_tx = msgs::msg::CANA();
    joint_tx.id = joint_node_write_ids[i];
        
    // Motor Stop Command
    joint_tx.data = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    rmd_can_publisher_->publish(joint_tx);

    // Brake Lock Command (don't think this works)
    joint_tx.data = {0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    rmd_can_publisher_->publish(joint_tx);
  }

  RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Successfully deactivated all RMD motors!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

double RMDHardwareInterface::calculate_joint_position_from_motor_position(double motor_position, int gear_ratio){
  // Converts from 0.01 deg to deg to radians/s
  return (motor_position * 0.01 * (M_PI/180.0))/gear_ratio;
}

double RMDHardwareInterface::calculate_joint_velocity_from_motor_velocity(double motor_velocity, int gear_ratio){
  // Converts from dps to radians/s
  return (motor_velocity * (M_PI/180.0))/gear_ratio;
}

int32_t RMDHardwareInterface::calculate_motor_position_from_desired_joint_position(double joint_position, int gear_ratio){
  // radians -> deg -> 0.01 deg
  return static_cast<int32_t>(std::round((joint_position*(180/M_PI)*100)*gear_ratio));
}

int32_t RMDHardwareInterface::calculate_motor_velocity_from_desired_joint_velocity(double joint_velocity, int gear_ratio){
  // radians/s -> deg/s -> 0.01 deg/s
  return static_cast<int32_t>(std::round((joint_velocity*(180/M_PI)*100)*gear_ratio));
}



hardware_interface::return_type RMDHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (rclcpp::ok())
  {
    rclcpp::spin_some(node_);
  }

  int data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  // Since you can't publish two messages in one sweep without a delay, every other iteration of read
  // will call angle or velocity messages
  current_iteration++;
  for(int i = 0; i < num_joints; i++) {
    auto joint_tx = msgs::msg::CANA();
    if(current_iteration%2 == 0){
      // Command to read multi-turn angle
      joint_tx.id = joint_node_write_ids[i];
      joint_tx.data = {0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    }
    else if(current_iteration%2 == 1){
      // Command to read motor status 2
      joint_tx.id = joint_node_write_ids[i];
      joint_tx.data = {0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    }
    rmd_can_publisher_->publish(joint_tx);
  }

  // Values retrieved
  for(int i = 0; i < num_joints; i++){  
    if(received_joint_data_.id == joint_node_read_ids[i] && received_joint_data_.data[0] == 0x9C){
      
      // DECODING CAN MESSAGE FOR VELOCITY
      data[0] = 0x9C;
      data[1] = received_joint_data_.data[1]; // Motor Temperature
      data[2] = received_joint_data_.data[2]; // Torque low byte
      data[3] = received_joint_data_.data[3]; // Torque high byte
      data[4] = received_joint_data_.data[4]; // speed low byte
      data[5] = received_joint_data_.data[5]; // speed high byte
      data[6] = received_joint_data_.data[6]; // encoder position low byte
      data[7] = received_joint_data_.data[7]; // encoder position high byte

      // ENCODER POSITION 
      // uint16 -> int16 -> double (for calcs)
      encoder_position = static_cast<double>(static_cast<int16_t>((data[7] << 8) | data[6]));

      // SPEED
      // uint16 -> int16 -> double (for calcs)
      motor_velocity = static_cast<double>(static_cast<int16_t>((data[5] << 8) | data[4]));

      // CALCULATING JOINT STATE
      joint_state_velocity_[i] = calculate_joint_velocity_from_motor_velocity(motor_velocity, joint_gear_ratios[i]);

    }
    else if(received_joint_data_.id == joint_node_read_ids[i] && received_joint_data_.data[0] == 0x92){
      
      // DECODING CAN MESSAGE FOR POSITION
      data[0] = 0x92;
      data[1] = 0x00;
      data[2] = 0x00;
      data[3] = 0x00;
      data[4] = received_joint_data_.data[4]; // Multi-turn low byte
      data[5] = received_joint_data_.data[5]; 
      data[6] = received_joint_data_.data[6];
      data[7] = received_joint_data_.data[7]; // Multi-turn high byte

      // POSITION
      // uint32 -> int32 -> double (for calcs)
      motor_position = static_cast<double>(static_cast<int32_t>((data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4]));

      if(DEBUG_MODE == 1) {
        for(int j = 0; j < 8; j++){
          RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Received at index %d: %d", j, received_joint_data_.data[j]);
        }
        RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Motor Position: %f", motor_position);
      }
      
      // CALCULATING JOINT STATE
      joint_state_position_[i] = calculate_joint_position_from_motor_position(motor_position, joint_gear_ratios[i]);

      // This initializes the command position for the joint (will not work if state isn't read immediately)
      // if(!joint_initialization_[i]){
      //   joint_command_position_[i] = joint_state_position_[i];
      //   joint_initialization_[i] = true;
      // }
    }
    else{
      if(DEBUG_MODE == 1) {
        RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Reply not heard.");
      }
    }

    if(DEBUG_MODE == 1) {
      RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Reading for joint: %s Motor Position: %f Joint position: %f Joint velocity: %f \nJoint Initialization State: %d\n", 
                                                        info_.joints[i].name.c_str(),
                                                        motor_position,
                                                        joint_state_position_[i], 
                                                        joint_state_velocity_[i],
                                                        static_cast<int>(joint_initialization_[i]));
    }

  }
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type rmd_ros2_control::RMDHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  int data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int32_t joint_angle = 0;
  int16_t operating_velocity = 200;
  int32_t joint_velocity = 0;
  
  for(int i = 0; i < num_joints; i++) {
    auto joint_tx = msgs::msg::CANA(); // Must reinstantiate else data from past iteration gets repeated
    joint_tx.id = joint_node_write_ids[i];
    
    if(control_level_[i] == integration_level_t::POSITION && std::isfinite(joint_command_position_[i])) {

      // CALCULATE DESIRED JOINT ANGLE
      joint_angle = joint_orientation[i]*calculate_motor_position_from_desired_joint_position(joint_command_position_[i], joint_gear_ratios[i]);

      if(DEBUG_MODE == 1) {
        RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Writing positions for: %s Joint angle of motor (0.01 deg): %d Joint command: %f", 
                                                              info_.joints[i].name.c_str(),
                                                              joint_angle,
                                                              joint_command_position_[i]);
      }
      
      // ENCODING CAN MESSAGE
      data[0] = 0xA4;
      data[1] = 0x00;
      data[2] = operating_velocity & 0xFF;
      data[3] = (operating_velocity >> 8) & 0xFF;
      data[4] = joint_angle & 0xFF;
      data[5] = (joint_angle >> 8) & 0xFF;
      data[6] = (joint_angle >> 16) & 0xFF;
      data[7] = (joint_angle >> 24) & 0xFF;
      
    }
    else if(control_level_[i] == integration_level_t::VELOCITY && std::isfinite(joint_command_velocity_[i])) {
      
      // CALCULATE DESIRED JOINT VELOCITY
      joint_velocity = joint_orientation[i]*calculate_motor_velocity_from_desired_joint_velocity(joint_command_velocity_[i], joint_gear_ratios[i]);

      if(DEBUG_MODE == 1) {
        RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Writing velocities for: %s Joint velocity of motor (0.01 dps): %d Joint command velocity: %f Orientation: %f CAN ID: %d", 
                                                        info_.joints[i].name.c_str(),
                                                        joint_velocity,
                                                        joint_command_velocity_[i],
                                                        joint_orientation[i],
                                                        joint_node_write_ids[i]);
      }
      
      // ENCODING CAN MESSAGE
      data[0] = 0xA2;
      data[1] = 0x00;
      data[2] = 0x00;
      data[3] = 0x00;  
      data[4] = joint_velocity & 0xFF;
      data[5] = (joint_velocity >> 8) & 0xFF;
      data[6] = (joint_velocity >> 16) & 0xFF;
      data[7] = (joint_velocity >> 24) & 0xFF;
    
    }
    else{
      RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Joint command value not found or undefined command state");
    }

    for(int j = 0; j < 8; j++){
      joint_tx.data.push_back(static_cast<uint8_t>(data[j]));
    }
    
    rmd_can_publisher_->publish(joint_tx);
  }
   


  return hardware_interface::return_type::OK;
}

// hardware_interface::return_type RMDHardwareInterface::perform_command_mode_switch(
//   const std::vector<std::string>& start_interfaces,
//   const std::vector<std::string>& stop_interfaces)
// {
//   std::vector<integration_level_t> new_modes = {};
//   for (std::string key : start_interfaces)
//   {
//     for (int i = 0; i < num_joints; i++){
//       if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION){
//         new_modes.push_back(integration_level_t::POSITION);
//       }
//       if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY){
//         new_modes.push_back(integration_level_t::VELOCITY);
//       }
//     }
//   }
//   // All joints must be given new command mode at the same time
//   // if (new_modes.size() != static_cast<unsigned long>(num_joints)){
//   //   return hardware_interface::return_type::ERROR;
//   // }
//   // All joints must have the same command mode
//   // if (!std::all_of(
//   //       new_modes.begin() + 1, new_modes.end(),
//   //       [&](integration_level_t mode) { return mode == new_modes[0]; }))
//   // {
//   //   return hardware_interface::return_type::ERROR;
//   // }

//   // Stop motion on all relevant joints that are stopping
//   for (std::string key : stop_interfaces) {
//     for (int i = 0; i < num_joints; i++) {
//       if (key.find(info_.joints[i].name) != std::string::npos) {
//         // fix this
//         joint_command_position_[i] = initial_position_[i];
//         joint_command_velocity_[i] = 0;
//         control_level_[i] = integration_level_t::UNDEFINED;  // Revert to undefined
//       }
//     }
//   }
//   // Set the new command modes. By this point everything should be undefined after the "stop motion" loop
//   for (int i = 0; i < num_joints; i++) {
//     if (control_level_[i] != integration_level_t::UNDEFINED) {
//       // Something else is using the joint! Abort!
//       return hardware_interface::return_type::ERROR;
//     }
//     control_level_[i] = new_modes[i];
//   }

//   return hardware_interface::return_type::OK;
// }

hardware_interface::return_type RMDHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string>& start_interfaces,
  const std::vector<std::string>& stop_interfaces)
{
  // Debug: print incoming requests
  std::ostringstream ss;
  ss << "perform_command_mode_switch called. start_interfaces: [";
  for (auto &s : start_interfaces) ss << s << ",";
  ss << "] stop_interfaces: [";
  for (auto &s : stop_interfaces) ss << s << ",";
  ss << "]";
  RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), ss.str().c_str());

  // For each joint, decide its new control mode based on start/stop interfaces.
  // We allow partial starts/stops: only affected joints are switched.
  std::vector<integration_level_t> requested_modes(num_joints, integration_level_t::UNDEFINED);

  // Process stop interfaces first: mark those joints as UNDEFINED
  for (const auto &ifname : stop_interfaces) {
    for (int i = 0; i < num_joints; ++i) {
      const std::string pos_if = info_.joints[i].name + "/" + std::string(hardware_interface::HW_IF_POSITION);
      const std::string vel_if = info_.joints[i].name + "/" + std::string(hardware_interface::HW_IF_VELOCITY);
      if (ifname == pos_if || ifname == vel_if || ifname.find(info_.joints[i].name) != std::string::npos) {
        requested_modes[i] = integration_level_t::UNDEFINED;
      }
    }
  }

  // Process start interfaces: set POSITION or VELOCITY per joint
  for (const auto &ifname : start_interfaces) {
    for (int i = 0; i < num_joints; ++i) {
      const std::string pos_if = info_.joints[i].name + "/" + std::string(hardware_interface::HW_IF_POSITION);
      const std::string vel_if = info_.joints[i].name + "/" + std::string(hardware_interface::HW_IF_VELOCITY);
      if (ifname == pos_if) {
        requested_modes[i] = integration_level_t::POSITION;
      } else if (ifname == vel_if) {
        requested_modes[i] = integration_level_t::VELOCITY;
      }
    }
  }

  // Now apply the requested_modes to control_level_.
  // For any joint with UNDEFINED in requested_modes, we only change it if it was explicitly stopped.
  for (int i = 0; i < num_joints; ++i) {
    if (requested_modes[i] == integration_level_t::UNDEFINED) {
      // if stop requested, set to UNDEFINED; otherwise leave existing mode
      // (we only set to UNDEFINED if this joint was mentioned in stop_interfaces)
      bool was_stopped = false;
      for (const auto &ifname : stop_interfaces) {
        if (ifname.find(info_.joints[i].name) != std::string::npos) {
          was_stopped = true;
          break;
        }
      }
      if (was_stopped) {
        control_level_[i] = integration_level_t::UNDEFINED;
        joint_command_velocity_[i] = 0;
        // optional: reset position cmd to current state to be safe
        joint_command_position_[i] = joint_state_position_[i];
        RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"),
          "Joint %s: stopped -> set UNDEFINED", info_.joints[i].name.c_str());
      }
      // else, leave control_level_ as-is
    } else {
      // Set the mode requested
      control_level_[i] = requested_modes[i];
      // If switching to velocity, optionally set command velocity to current state to avoid jumps
      if (requested_modes[i] == integration_level_t::VELOCITY) {
        // joint_command_velocity_[i] = joint_state_velocity_[i];
        joint_command_velocity_[i] = 0;
        RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"),
          "Joint %s: switched to VELOCITY (cmd vel initialized from state: %f)",
          info_.joints[i].name.c_str(), joint_command_velocity_[i]);
      } else if (requested_modes[i] == integration_level_t::POSITION) {
        joint_command_position_[i] = joint_state_position_[i];
        RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"),
          "Joint %s: switched to POSITION (cmd pos initialized from state: %f)",
          info_.joints[i].name.c_str(), joint_command_position_[i]);
      }
    }
  }

  return hardware_interface::return_type::OK;
}


}  // namespace rmd_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rmd_ros2_control::RMDHardwareInterface, hardware_interface::SystemInterface)

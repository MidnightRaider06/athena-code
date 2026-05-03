#pragma once

#include <string>
#include <memory>
#include <chrono>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class ObjectDetected : public BT::ConditionNode
{
public:
  ObjectDetected(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void callback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;

  bool target_found_;
  rclcpp::Time last_msg_time_;
  bool received_msg_;
};
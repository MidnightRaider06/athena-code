#include "athena_planner/object_detected_node.hpp"

#include <string>

namespace athena_planner
{

ObjectDetected::ObjectDetected(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config),
  target_found_(false),
  received_msg_(false)
{
  node_ = rclcpp::Node::make_shared("object_detected_bt_node");

  std::string object_topic;
  getInput("object_topic", object_topic);

  sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    object_topic,
    rclcpp::QoS(10),
    std::bind(&ObjectDetected::targetFoundCallback, this, std::placeholders::_1));
}

BT::PortsList ObjectDetected::providedPorts()
{
  return {
    BT::InputPort<std::string>("object_topic", "/yolo/target_found",
      "Topic publishing whether the target object is detected"),
    BT::InputPort<double>("timeout", 1.0,
      "Max allowed age of the target_found message")
  };
}

void ObjectDetected::targetFoundCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  target_found_ = msg->data;
  received_msg_ = true;
  last_msg_time_ = node_->now();
}

BT::NodeStatus ObjectDetected::tick()
{
  rclcpp::spin_some(node_);

  if (!received_msg_) {
    return BT::NodeStatus::FAILURE;
  }

  double timeout = 1.0;
  getInput("timeout", timeout);

  const double age = (node_->now() - last_msg_time_).seconds();
  if (age > timeout) {
    return BT::NodeStatus::FAILURE;
  }

  return target_found_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace athena_planner

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_planner::ObjectDetected>("ObjectDetected");
}
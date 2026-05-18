#include "athena_planner/led_status_node.hpp"

namespace bt_nodes
{

LedStatusNode::LedStatusNode(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  topic_name_ = "/led_status";
  getInput("topic_name", topic_name_);

  led_pub_ = node_->create_publisher<msgs::msg::LedStatus>(
    topic_name_,
    10);
}

BT::NodeStatus LedStatusNode::tick()
{
  std::string color;
  if (!getInput("color", color)) {
    RCLCPP_ERROR(node_->get_logger(), "LedStatusNode missing required input port [color]");
    return BT::NodeStatus::FAILURE;
  }

  msgs::msg::LedStatus msg;
  msg.cmd = msgs::msg::LedStatus::CMD_SOLID;
  msg.r = 0;
  msg.g = 0;
  msg.b = 0;
  msg.param = 0;

  if (color == "red" || color == "RED") {
    msg.cmd = msgs::msg::LedStatus::CMD_SOLID;
    msg.r = 255;
    msg.g = 0;
    msg.b = 0;
  } else if (color == "green" || color == "GREEN") {
    msg.cmd = msgs::msg::LedStatus::CMD_FLASH;
    msg.r = 0;
    msg.g = 255;
    msg.b = 0;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Invalid LED color [%s]. Expected red or green.",
      color.c_str());
    return BT::NodeStatus::FAILURE;
  }

  led_pub_->publish(msg);

  RCLCPP_INFO(
    node_->get_logger(),
    "Published LED status: color=%s rgb=[%u, %u, %u]",
    color.c_str(),
    msg.r,
    msg.g,
    msg.b);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nodes
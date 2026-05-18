#ifndef ATHENA_PLANNER__LED_STATUS_NODE_HPP_
#define ATHENA_PLANNER__LED_STATUS_NODE_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

// Change this include if your msg package name is different
#include "msgs/msg/led_status.hpp"

namespace bt_nodes
{

class LedStatusNode : public BT::SyncActionNode
{
public:
  LedStatusNode(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("color"),
      BT::InputPort<std::string>("topic_name", "/led_status")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<msgs::msg::LedStatus>::SharedPtr led_pub_;

  std::string topic_name_;
};

}  // namespace bt_nodes

#endif  // ATHENA_PLANNER__LED_STATUS_NODE_HPP_
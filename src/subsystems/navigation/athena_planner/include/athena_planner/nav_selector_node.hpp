#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__NAV_SELECTOR_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__NAV_SELECTOR_NODE_HPP_

#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"

namespace bt_nodes
{

class NavSelector : public BT::SyncActionNode
{
public:
 
  NavSelector(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "default_nav_mode",
        "the default nav mode to use if there is not any external topic message received."),

      BT::InputPort<std::string>(
        "topic_name",
        "nav_selector",
        "the input topic name to select the nav mode"),

      BT::OutputPort<std::string>(
        "nav_mode",
        "Selected nav mode by subscription")
    };
  }

private:
  
  BT::NodeStatus tick() override;

  void callbackNavSelect(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_selector_sub_;

  std::string last_selected_nav_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::string topic_name_;
};

}  

#endif 

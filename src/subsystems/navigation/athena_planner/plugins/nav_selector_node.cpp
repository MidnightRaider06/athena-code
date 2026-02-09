
#include <string>
#include <memory>

#include "std_msgs/msg/string.hpp"

#include "athena_planner/nav_selector_node.hpp"

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace bt_nodes
{

using std::placeholders::_1;

NavSelector::NavSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  getInput("topic_name", topic_name_);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  nav_selector_sub_ = node_->create_subscription<std_msgs::msg::String>(
    topic_name_,
    qos,
    std::bind(&NavSelector::callbackNavSelect, this, _1),
    sub_option);
}

BT::NodeStatus NavSelector::tick()
{
  callback_group_executor_.spin_some();

  
  if (last_selected_nav_.empty()) {
    std::string default_nav_mode;
    getInput("default_nav_mode", default_nav_mode);
    if (default_nav_mode.empty()) {
      return BT::NodeStatus::FAILURE;
    } else {
      last_selected_nav_ = default_nav_mode;
    }
  }

  setOutput("nav_mode", last_selected_nav_);

  return BT::NodeStatus::SUCCESS;
}

void
NavSelector::callbackNavSelect(const std_msgs::msg::String::SharedPtr msg)
{
  last_selected_nav_ = msg->data;
}

}  

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_nodes::NavSelector>("NavSelector");
}

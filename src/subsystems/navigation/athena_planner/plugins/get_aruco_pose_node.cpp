#include "athena_planner/get_aruco_pose_node.hpp"

namespace bt_nodes
{

using std::placeholders::_1;

GetArucoPose::GetArucoPose(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf),
  has_pose_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  getInput("aruco_topic", topic_name_);

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  
  aruco_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    topic_name_,
    10,
    std::bind(&GetArucoPose::arucoCallback, this, _1),
    sub_option);
}

BT::NodeStatus GetArucoPose::tick()
{
  // Spin to process callbacks
  callback_group_executor_.spin_some();

  if (!has_pose_) {
    RCLCPP_WARN(node_->get_logger(), "No ArUco pose available yet");
    return BT::NodeStatus::FAILURE;
  }

  // Output the latest pose to the blackboard
  setOutput("aruco_pose", latest_pose_);

  RCLCPP_INFO(node_->get_logger(), 
    "Retrieved ArUco pose: [%.2f, %.2f, %.2f]",
    latest_pose_.pose.position.x,
    latest_pose_.pose.position.y,
    latest_pose_.pose.position.z);

  return BT::NodeStatus::SUCCESS;
}

void GetArucoPose::arucoCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  latest_pose_ = *msg;
  has_pose_ = true;
}

}  // namespace bt_nodes
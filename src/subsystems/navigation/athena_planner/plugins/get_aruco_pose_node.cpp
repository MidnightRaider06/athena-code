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
  
  timeout_ = 1.5;
  getInput("aruco_topic", topic_name_);
  getInput("timeout", timeout_);

  last_pose_stamp_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  
  aruco_sub_ = node_->create_subscription<vision_msgs::msg::Detection2D>(
    topic_name_,
    10,
    std::bind(&GetArucoPose::arucoCallback, this, _1),
    sub_option);
}

BT::NodeStatus GetArucoPose::tick()
{
  RCLCPP_INFO(node_->get_logger(), "GetArucoPose::tick() called");
  
  // Spin to process callbacks
  callback_group_executor_.spin_some();

  if (!has_pose_) {
    RCLCPP_WARN(node_->get_logger(), "No ArUco pose available yet");
    return BT::NodeStatus::FAILURE;
  }

  // Reject stale poses so we don't reuse an old ArUco goal
  auto current_time = node_->now();
  double time_since_detection = (current_time - last_pose_stamp_).seconds();

  if (time_since_detection > timeout_) {
    RCLCPP_WARN(node_->get_logger(),
      "ArUco pose stale (%.2fs > %.2fs), ignoring", time_since_detection, timeout_);
    has_pose_ = false;
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
  const vision_msgs::msg::Detection2D::SharedPtr msg)
{
  latest_pose_.header = msg->header;
  latest_pose_.pose = msg->results[0].pose.pose;
  last_pose_stamp_ = rclcpp::Time(msg->header.stamp);
  has_pose_ = true;
}

}  // namespace bt_nodes
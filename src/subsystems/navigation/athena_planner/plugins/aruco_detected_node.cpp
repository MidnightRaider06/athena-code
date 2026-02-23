#include "athena_planner/aruco_detected_node.hpp"

namespace bt_nodes
{

using std::placeholders::_1;

ArUcoDetected::ArUcoDetected(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  
  timeout_ = 1.0; // default detection timeout
  getInput("aruco_topic", topic_name_);
  getInput("timeout", timeout_);

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  
  aruco_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    topic_name_,
    10,
    std::bind(&ArUcoDetected::arucoCallback, this, _1),
    sub_option);

  last_detection_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
}

BT::NodeStatus ArUcoDetected::tick()
{
  // RCLCPP_INFO(node_->get_logger(), "ArUcoDetected::tick() called");

  // Spin to process callbacks
  callback_group_executor_.spin_some();

  if (last_detection_time_.nanoseconds() == 0) {
    // RCLCPP_INFO(node_->get_logger(), "ArUco never detected");
    return BT::NodeStatus::FAILURE;
  }

  // Check if detection is recent enough
  auto current_time = node_->now();
  double time_since_detection = (current_time - last_detection_time_).seconds();

  // RCLCPP_INFO(node_->get_logger(), "Current time: %.2f", current_time.seconds());
  // RCLCPP_INFO(node_->get_logger(), "Last detection time: %.2f", last_detection_time_.seconds());

  if (time_since_detection < timeout_) {
    // RCLCPP_INFO(node_->get_logger(), "ArUco DETECTED (%.2fs ago)", time_since_detection);
    return BT::NodeStatus::SUCCESS;
  } else {
    // RCLCPP_INFO(node_->get_logger(), "ArUco too old (%.2fs ago)", time_since_detection);
    return BT::NodeStatus::FAILURE;
  }
}

void ArUcoDetected::arucoCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  (void)msg;  // Mark as used
  last_detection_time_ = rclcpp::Time(msg->header.stamp);
}

}  // namespace bt_nodes
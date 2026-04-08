#ifndef ATHENA_PLANNER__ARUCO_DETECTED_NODE_HPP_
#define ATHENA_PLANNER__ARUCO_DETECTED_NODE_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "vision_msgs/msg/detection2_d.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace bt_nodes
{

/**
 * @brief Condition node that checks if an ArUco marker has been detected recently
 * 
 * Monitors the aruco_pose topic and returns SUCCESS if a message was received
 * within the timeout period, FAILURE otherwise.
 */
class ArUcoDetected : public BT::ConditionNode
{
public:
  ArUcoDetected(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "aruco_topic",
        "/aruco_detection",
        "Topic name for ArUco detection messages"),
      BT::InputPort<double>(
        "timeout",
        1.0,
        "Time in seconds to consider ArUco detection valid")
    };
  }

private:
  BT::NodeStatus tick() override;

  void arucoCallback(const vision_msgs::msg::Detection2D::SharedPtr msg);

  rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr aruco_sub_;
  
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::string topic_name_;
  double timeout_;
  
  rclcpp::Time last_detection_time_;
};

}  // namespace bt_nodes

#endif  // ATHENA_PLANNER__ARUCO_DETECTED_NODE_HPP_
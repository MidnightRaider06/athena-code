#ifndef ATHENA_PLANNER__GET_ARUCO_POSE_NODE_HPP_
#define ATHENA_PLANNER__GET_ARUCO_POSE_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace bt_nodes
{

/**
 * @brief Action node that retrieves the latest ArUco marker pose
 * 
 * Subscribes to the aruco_pose topic and outputs the most recent pose
 * to the blackboard for use by navigation nodes.
 */
class GetArucoPose : public BT::SyncActionNode
{
public:
  GetArucoPose(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "aruco_topic",
        "/aruco_pose",
        "Topic name for ArUco pose messages"),
      BT::InputPort<std::string>(
        "timeout",
        "1.5",
        "Time in seconds to consider ArUco detection valid"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "aruco_pose",
        "The retrieved ArUco marker pose")
    };
  }

private:
  BT::NodeStatus tick() override;

  void arucoCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_sub_;
  
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::string topic_name_;
  double timeout_;
  rclcpp::Time last_pose_stamp_;
  
  geometry_msgs::msg::PoseStamped latest_pose_;
  bool has_pose_;

};

}  // namespace bt_nodes

#endif  // ATHENA_PLANNER__GET_ARUCO_POSE_NODE_HPP_
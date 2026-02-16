#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SPIRAL_COVERAGE_ACTION_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SPIRAL_COVERAGE_ACTION_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_nodes
{

/**
 * BT action node to generate an Archimedean spiral coverage path
 * 
 * Generates a spiral search pattern for coverage tasks. The spiral expands
 * outward from the robot's current position (or specified center) up to a
 * maximum radius, with configurable spacing between loops.
 * 
 * Input Ports:
 *   radius         [double] Maximum radius of the spiral in meters (default: 15.0)
 *   spacing        [double] Distance between spiral loops in meters (default: 2.0)
 *   angular_step   [double] Angular resolution in radians (default: 0.1)
 *   frame_id       [string] Reference frame for the path (default: "map")
 * 
 * Output Ports:
 *   path           [nav_msgs::msg::Path] The generated spiral coverage path
 */
class SpiralCoverageAction : public BT::SyncActionNode
{
public:

  SpiralCoverageAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:

  nav_msgs::msg::Path generateSpiralPath(
    double radius,
    double spacing,
    double angular_step,
    const std::string & frame_id);

  double calculateTangentAngle(double theta);
};

}

#endif
#include "athena_planner/spiral_coverage_action_node.hpp"

#include <cmath>
#include <vector>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bt_nodes
{

    SpiralCoverageAction::SpiralCoverageAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
    : BT::SyncActionNode(xml_tag_name, conf)
    {
    }

    BT::PortsList SpiralCoverageAction::providedPorts()
    {
    return {
        BT::InputPort<double>("radius", 15.0, "Maximum radius of spiral in meters"),
        BT::InputPort<double>("spacing", 2.0, "Distance between spiral loops in meters"),
        BT::InputPort<double>("angular_step", 0.1, "Angular step size in radians"),
        BT::InputPort<std::string>("frame_id", "map", "Frame ID for the path"),
        BT::OutputPort<nav_msgs::msg::Path>("path", "Generated spiral path")
    };
    }

    BT::NodeStatus SpiralCoverageAction::tick() {
    // Get input parameters with defaults
    double radius = 15.0;
    double spacing = 2.0;
    double angular_step = 0.1;
    std::string frame_id = "map";

    getInput("radius", radius);
    getInput("spacing", spacing);
    getInput("angular_step", angular_step);
    getInput("frame_id", frame_id);

    // Generate the spiral path
    nav_msgs::msg::Path spiral_path = generateSpiralPath(
        radius, spacing, angular_step, frame_id);

    if (spiral_path.poses.empty()) {
        RCLCPP_ERROR(
        rclcpp::get_logger("SpiralCoverageAction"),
        "Failed to generate spiral path");
        return BT::NodeStatus::FAILURE;
    }

    // Set output
    setOutput("path", spiral_path);

    RCLCPP_INFO(
        rclcpp::get_logger("SpiralCoverageAction"),
        "Generated spiral path with %zu waypoints (radius: %.2fm, spacing: %.2fm)",
        spiral_path.poses.size(), radius, spacing);

    return BT::NodeStatus::SUCCESS;
    }

    nav_msgs::msg::Path SpiralCoverageAction::generateSpiralPath(
    double radius,
    double spacing,
    double angular_step,
    const std::string & frame_id)
    {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = rclcpp::Clock().now();

    // Archimedean spiral equation: r = a * θ
    // For uniform spacing 's' between loops: a = s / (2π)
    const double a = spacing / (2.0 * M_PI);

    // Calculate maximum angle needed to reach the desired radius
    // From r = a * θ, θ_max = r_max / a
    const double max_angle = (a > 0.0) ? (radius / a) : 0.0;

    if (max_angle <= 0.0) {
        RCLCPP_WARN(
        rclcpp::get_logger("SpiralCoverageAction"),
        "Max angle is non-positive: %f", max_angle);
        return path;
    }

    // Start at a minimum radius to avoid sharp initial turns
        const double min_start_radius = 1.0;
        const double theta_start = (a > 0.0) ? (min_start_radius / a) : 0.0;

        double theta = theta_start;  // Start from minimum radius instead of center
        while (theta <= max_angle) {

            const double r = a * theta;

            // Stop if we've exceeded the maximum radius
            if (r > radius) {
            break;
        }

        // Polar to cartesian
        const double x = r * std::cos(theta);
        const double y = r * std::sin(theta);

        // Calculate orientation tangent to spiral
        const double orientation_angle = calculateTangentAngle(theta);


        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = path.header.stamp;

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, orientation_angle);
        pose.pose.orientation = tf2::toMsg(q);

        path.poses.push_back(pose);

        // Increment angle for next waypoint
        theta += angular_step;
    }

    return path;
    }

    double SpiralCoverageAction::calculateTangentAngle(double theta)
    {
    if (theta < 1e-6) {
        return 2.0 * theta;
    }
    
    return theta + std::atan(theta);
    }

}  // namespace bt_nodes

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
        : BT::StatefulActionNode(xml_tag_name, conf),
        path_generated_(false)
    {
        tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    }

    BT::PortsList SpiralCoverageAction::providedPorts()
    {
        return {
            BT::InputPort<double>("radius", 15.0, "Maximum radius of spiral in meters"),
            BT::InputPort<double>("spacing", 2.0, "Distance between spiral loops in meters"),
            BT::InputPort<double>("angular_step", 0.5, "Angular step size in radians"),
            BT::InputPort<std::string>("frame_id", "map", "Frame ID for the path"),
            BT::OutputPort<nav_msgs::msg::Path>("path", "Generated spiral path")
        };
    }

    BT::NodeStatus SpiralCoverageAction::onStart()
    {
        // Generate once when this node becomes active
        path_generated_ = false;
        cached_path_.poses.clear();

        double radius = 15.0, spacing = 2.0, angular_step = 0.1;
        std::string frame_id = "map";
        getInput("radius", radius);
        getInput("spacing", spacing);
        getInput("angular_step", angular_step);
        getInput("frame_id", frame_id);

        geometry_msgs::msg::PoseStamped current_pose;
        if (!nav2_util::getCurrentPose(current_pose, *tf_buffer_, frame_id)) {
            RCLCPP_ERROR(rclcpp::get_logger("SpiralCoverageAction"), "Failed to get current pose");
            return BT::NodeStatus::FAILURE;
        }

        cached_path_ = generateSpiralPath(radius, spacing, angular_step, frame_id, current_pose);
        if (cached_path_.poses.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("SpiralCoverageAction"), "Generated spiral path is empty");
            return BT::NodeStatus::FAILURE;
        }

        path_generated_ = true;
        setOutput("path", cached_path_);

        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus SpiralCoverageAction::onRunning()
    {
        // If this node is ticked again while still active, just output cached path
        if (path_generated_) {
            setOutput("path", cached_path_);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
        }

        void SpiralCoverageAction::onHalted()
        {
        // Called when coverage branch is preempted (e.g., ArUco detected)
        path_generated_ = false;
        cached_path_.poses.clear();
    }


    nav_msgs::msg::Path SpiralCoverageAction::generateSpiralPath(
        double radius,
        double spacing,
        double angular_step,
        const std::string & frame_id,
        const geometry_msgs::msg::PoseStamped & current_pose)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = frame_id;
        path.header.stamp = rclcpp::Clock().now();

        // Archimedean spiral equation: r = a * θ
        // For uniform spacing 's' between loops: a = s / (2π)
        const double a = spacing / (2.0 * M_PI);
        const double yaw0 = tf2::getYaw(current_pose.pose.orientation);

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
        const double min_start_radius = 1.5;
        double theta = 0.0;  // Start from minimum radius instead of center
        while (theta <= max_angle) {

            const double r = min_start_radius + (a * theta);

            // Stop if we've exceeded the maximum radius
            if (r > radius) {
                break;
            }

            // Polar to cartesian
            const double x = current_pose.pose.position.x + r * std::cos(yaw0 + theta + M_PI_2);
            const double y = current_pose.pose.position.y + r * std::sin(yaw0 + theta + M_PI_2);

            // Calculate orientation tangent to spiral
            const double orientation_angle = yaw0 + theta;


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

}  // namespace bt_nodes

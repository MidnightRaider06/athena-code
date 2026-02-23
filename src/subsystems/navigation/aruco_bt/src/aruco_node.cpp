#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ArUcoNode : public rclcpp::Node
{
public:
    ArUcoNode() : Node("aruco_node")
    {
        this->declare_parameter("marker_size", 0.2);
        
        bool sim = this->get_parameter("use_sim_time").as_bool();
        marker_size_ = this->get_parameter("marker_size").as_double();

        if (sim) {
            marker_size_ = 0.50;
        }
        
        std::string image_topic = "/zed/zed_node/left/image_rect_color";
        std::string camera_info_topic = "/zed/zed_node/left/camera_info";

        RCLCPP_INFO(this->get_logger(), "sim=%s", sim ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "marker_size=%.3f meters", marker_size_);
        RCLCPP_INFO(this->get_logger(), "Subscribing to image feed: %s", image_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribing to camera info: %s", camera_info_topic.c_str());

        aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        aruco_params_ = cv::aruco::DetectorParameters::create();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, 10,
            std::bind(&ArUcoNode::imageCallback, this, std::placeholders::_1));
        
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, 10,
            std::bind(&ArUcoNode::cameraInfoCallback, this, std::placeholders::_1));

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/aruco_annotated_img", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco_pose", 10);

        marker_id_ = -1;
        camera_info_received_ = false;
    }

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (!camera_info_received_) {
            // Extract camera matrix (K) and distortion coefficients
            camera_matrix_ = cv::Mat(3, 3, CV_64F);
            for (int i = 0; i < 9; i++) {
                camera_matrix_.at<double>(i / 3, i % 3) = msg->k[i];
            }

            dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F);
            for (size_t i = 0; i < msg->d.size(); i++) {
                dist_coeffs_.at<double>(i) = msg->d[i];
            }

            camera_info_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera info received");
            camera_info_sub_.reset();
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS image to OpenCV format
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Convert grayscale to color for annotation
        cv::Mat frame_color;
        cv::cvtColor(cv_ptr->image, frame_color, cv::COLOR_GRAY2BGR);

        // Detect ArUco markers
        detectArucoMarkers(cv_ptr->image);

        // If markers detected, annotate the image and estimate pose
        if (marker_id_ >= 0 && !corners_.empty()) {
            // Draw bounding box around marker
            for (size_t i = 0; i < 4; i++) {
                cv::Point start_point(static_cast<int>(corners_[i].x), 
                                     static_cast<int>(corners_[i].y));
                cv::Point end_point(static_cast<int>(corners_[(i + 1) % 4].x), 
                                   static_cast<int>(corners_[(i + 1) % 4].y));
                cv::line(frame_color, start_point, end_point, cv::Scalar(0, 255, 0), 2);
            }

            // Add text label with marker ID
            cv::Point text_pos(static_cast<int>(corners_[0].x), 
                              static_cast<int>(corners_[0].y) - 20);
            std::string label = "id: " + std::to_string(marker_id_);
            cv::putText(frame_color, label, text_pos, 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                       cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

            // Estimate pose if camera info is available
            if (camera_info_received_) {
                estimateAndPublishPose(msg);
                
                // Draw axis on the image to visualize pose
                cv::aruco::drawAxis(frame_color, camera_matrix_, dist_coeffs_, 
                                   rvec_, tvec_, marker_size_ * 0.5);
            }
        }

        // Convert back to ROS message and publish
        sensor_msgs::msg::Image::SharedPtr annotated_msg = 
            cv_bridge::CvImage(msg->header, "bgr8", frame_color).toImageMsg();
        image_pub_->publish(*annotated_msg);
    }

    void detectArucoMarkers(const cv::Mat& frame)
    {
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;

        cv::aruco::detectMarkers(frame, aruco_dict_, corners, ids, aruco_params_);

        if (!ids.empty()) {
            // Store all corners for the first detected marker
            marker_id_ = ids[0];
            all_corners_.clear();
            all_corners_.push_back(corners[0]);
            
            corners_.clear();
            for (const auto& pt : corners[0]) {
                corners_.push_back(pt);
            }
        } else {
            marker_id_ = -1;
            corners_.clear();
            all_corners_.clear();
        }
    }

    void estimateAndPublishPose(const sensor_msgs::msg::Image::SharedPtr& msg)
    {
        if (all_corners_.empty()) {
            return;
        }

        if (marker_size_ <= 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid marker_size_: %.3f", marker_size_);
            return;
        }

        std::vector<cv::Vec3d> rvecs, tvecs;
        
        // Estimate pose of the marker
        cv::aruco::estimatePoseSingleMarkers(all_corners_, marker_size_, 
                                            camera_matrix_, dist_coeffs_, 
                                            rvecs, tvecs);

        // Store for visualization
        rvec_ = rvecs[0];
        tvec_ = tvecs[0];

        // Convert rotation vector to rotation matrix, then to quaternion
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec_, rotation_matrix);
        rotation_matrix.convertTo(rotation_matrix, CV_64F);

        // Convert rotation matrix to quaternion (tf2)
        tf2::Matrix3x3 m(
          rotation_matrix.at<double>(0,0), rotation_matrix.at<double>(0,1), rotation_matrix.at<double>(0,2),
          rotation_matrix.at<double>(1,0), rotation_matrix.at<double>(1,1), rotation_matrix.at<double>(1,2),
          rotation_matrix.at<double>(2,0), rotation_matrix.at<double>(2,1), rotation_matrix.at<double>(2,2)
        );
        tf2::Quaternion q;
        m.getRotation(q);
        q.normalize();

        // Create PoseStamped in camera frame
        geometry_msgs::msg::PoseStamped pose_camera;
        pose_camera.header.stamp = msg->header.stamp;
        pose_camera.header.frame_id = msg->header.frame_id;

        // Position (translation vector from camera to marker)
        pose_camera.pose.position.x = tvec_[0];
        pose_camera.pose.position.y = tvec_[1];
        pose_camera.pose.position.z = tvec_[2];

        // Orientation (quaternion)
        pose_camera.pose.orientation.x = 0.0;
        pose_camera.pose.orientation.y = 0.0;
        pose_camera.pose.orientation.z = 0.0;
        pose_camera.pose.orientation.w = 1.0;


        // Transform to map frame
        try {
            geometry_msgs::msg::PoseStamped pose_map;
            tf_buffer_->transform(pose_camera, pose_map, "map", tf2::durationFromSec(0.1));

            geometry_msgs::msg::TransformStamped tf_map_base =
            tf_buffer_->lookupTransform("map", "base_footprint", msg->header.stamp, tf2::durationFromSec(0.1));
            
            // Overwrite goal orientation with rover's current orientation
            pose_map.pose.orientation.x = tf_map_base.transform.rotation.x;
            pose_map.pose.orientation.y = tf_map_base.transform.rotation.y;
            pose_map.pose.orientation.z = tf_map_base.transform.rotation.z;
            pose_map.pose.orientation.w = tf_map_base.transform.rotation.w;

            pose_pub_->publish(pose_map);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform to map frame: %s", ex.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool camera_info_received_;
    double marker_size_;

    int marker_id_;
    std::vector<cv::Point2f> corners_;
    std::vector<std::vector<cv::Point2f>> all_corners_;
    
    cv::Vec3d rvec_;  // Rotation vector
    cv::Vec3d tvec_;  // Translation vector
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArUcoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
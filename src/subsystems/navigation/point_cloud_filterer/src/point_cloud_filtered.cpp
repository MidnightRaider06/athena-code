#include <chrono>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

class PointCloudFilterer : public rclcpp::Node {
public:
  PointCloudFilterer()
  : Node("point_cloud_filterer") {

    input_topic_  = declare_parameter<std::string>("input_topic", "/depth_camera/points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/depth_camera/points_corrected");

    frame_override_ = declare_parameter<std::string>("frame_override", "");

    output_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS{});

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PointCloudFilterer::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Point Cloud Filterer initialized");
    RCLCPP_INFO(get_logger(), "  Input:  %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Output: %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Frame override: %s",
                frame_override_.empty() ? "(none - using message frame_id)" : frame_override_.c_str());
  }

private:

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
      "Received point cloud - frame_id: '%s', size: %zu bytes",
      msg->header.frame_id.c_str(), msg->data.size());

    auto output_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);

    processPointCloud(output_msg);

    output_pub_->publish(*output_msg);
  }

  void processPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    correctFrameHeader(cloud);

    // add more steps here maybe
  }

  void correctFrameHeader(sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    if (!frame_override_.empty()) {
      std::string original_frame = cloud->header.frame_id;
      cloud->header.frame_id = frame_override_;

      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
        "Corrected frame_id: '%s' -> '%s'",
        original_frame.c_str(), frame_override_.c_str());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_override_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFilterer>());
  rclcpp::shutdown();
  return 0;
}

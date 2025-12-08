#include <chrono>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>   // tf2::doTransform for PointCloud2
#include <tf2_eigen/tf2_eigen.hpp>               // transformToEigen

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

// ZED plane message (package: zed msgs)
#include <zed_msgs/msg/plane_stamped.hpp>

// Run command below to publish camera -> base link transform
// ros2 run tf2_ros static_transform_publisher \
//   0 0.30 0 \
//   -0.5 0.5 -0.5 -0.5 \
//   camera_link base_link

using namespace std::chrono_literals;

class ObstacleFilterNode : public rclcpp::Node {
public:
  ObstacleFilterNode() 
  : Node("obstacle_filter"), 
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_){

    input_topic_  = declare_parameter<std::string>("input_topic", "/zed/zed_node/point_cloud/cloud_registered");
    output_topic_ = declare_parameter<std::string>("output_topic", "/obstacles/points");

    min_range_ = declare_parameter<double>("min_range_m", 0.4);
    max_range_ = declare_parameter<double>("max_range_m", 6.0);
    min_height_= declare_parameter<double>("min_height_m", 0.05);
    max_height_= declare_parameter<double>("max_height_m", 1.00);
    voxel_leaf_ = declare_parameter<double>("voxel_leaf_m", 0.08);

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS{});
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ObstacleFilterNode::cbCloud, this, std::placeholders::_1));

    clicked_point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/clicked_point", 10);
    ground_plane_sub_ = create_subscription<zed_msgs::msg::PlaneStamped>(
      "/plane", rclcpp::QoS(10),
      std::bind(&ObstacleFilterNode::onPlane, this, std::placeholders::_1));

    // ---- 1 Hz timer publishes (0, 5, 5) in camera_link ----
    timer_ = create_wall_timer(1s, std::bind(&ObstacleFilterNode::onTimer, this));
  }

private:

  struct PlaneCache {
    rclcpp::Time stamp;
    std::string frame_id;       // frame of plane fields (from header.frame_id)
    Eigen::Vector3f n_cam;      // normal in plane frame (usually camera)
    Eigen::Vector3f c_cam;      // a point on plane in plane frame
    bool valid{false};
  };

  // 1 Hz: send the click ray point
  void onTimer() {
    geometry_msgs::msg::PointStamped pt;
    pt.header.stamp = now();
    pt.header.frame_id = "camera_link"; // publish in camera frame
    // Optical frame: X right, Y down, Z forward → “forward & down” = (0, +5, +5)
    pt.point.x = 0.0;
    pt.point.y = 5.0;
    pt.point.z = 5.0;
    clicked_point_pub_->publish(pt);
  }

  void onPlane(const zed_msgs::msg::PlaneStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(plane_mtx_);
    latest_plane_.stamp    = msg->header.stamp;
    latest_plane_.frame_id = msg->header.frame_id;

    // PlaneStamped.msg: normal & center are geometry_msgs/Point32
    latest_plane_.n_cam = Eigen::Vector3f(
      msg->normal.x, msg->normal.y, msg->normal.z);
    latest_plane_.c_cam = Eigen::Vector3f(
      msg->center.x, msg->center.y, msg->center.z);

    // Normalize normal
    float n = latest_plane_.n_cam.norm();
    if (n > 1e-6f) latest_plane_.n_cam /= n;

    latest_plane_.valid = true;
  }

  void cbCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Transform cloud → base_link
    sensor_msgs::msg::PointCloud2 cloud_base;
    try {
      auto tf = tf_buffer_.lookupTransform(
        "base_link", msg->header.frame_id, msg->header.stamp, 50ms);
      tf2::doTransform(*msg, cloud_base, tf);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF (cloud) failed: %s", ex.what());
      return;
    }

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_base, *in);

    // Downsample
    pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(in);
    voxel.setLeafSize(static_cast<float>(voxel_leaf_), static_cast<float>(voxel_leaf_), static_cast<float>(voxel_leaf_));
    voxel.filter(*ds);

    // Range gate
    pcl::PointCloud<pcl::PointXYZ>::Ptr ranged(new pcl::PointCloud<pcl::PointXYZ>);
    ranged->reserve(ds->size());
    const float min_r2 = static_cast<float>(min_range_ * min_range_);
    const float max_r2 = static_cast<float>(max_range_ * max_range_);
    for (const auto &p : ds->points) {
      float r2 = p.x*p.x + p.y*p.y + p.z*p.z;
      if (r2 >= min_r2 && r2 <= max_r2) ranged->push_back(p);
    }

    // Transform latest plane → base_link, then filter by signed height
    Eigen::Vector3f n_base(0.f, 0.f, 1.f);
    float d_base = 0.f;
    bool have_plane = false;

    {
      std::lock_guard<std::mutex> lock(plane_mtx_);
      if (latest_plane_.valid) {
        try {
          auto tfp = tf_buffer_.lookupTransform(
            "base_link", latest_plane_.frame_id, latest_plane_.stamp, 50ms);
          Eigen::Isometry3d T = tf2::transformToEigen(tfp.transform);
          Eigen::Matrix3f R = T.rotation().cast<float>();
          Eigen::Vector3f t = T.translation().cast<float>();

          n_base = (R * latest_plane_.n_cam).normalized();
          Eigen::Vector3f c_base = R * latest_plane_.c_cam + t;

          // Orient normal upward if needed (helps consistency)
          if (n_base.z() < 0.f) n_base = -n_base;

          d_base = -n_base.dot(c_base);
          have_plane = true;
        } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF (plane) failed: %s", ex.what());
        }
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr band(new pcl::PointCloud<pcl::PointXYZ>);
    band->reserve(ranged->size());

    if (have_plane) {
      const float hmin = static_cast<float>(min_height_);
      const float hmax = static_cast<float>(max_height_);
      for (const auto &p : ranged->points) {
        float h = n_base.x()*p.x + n_base.y()*p.y + n_base.z()*p.z + d_base;
        if (h >= hmin && h <= hmax) band->push_back(p);
      }
    } else {
      // Fallback: simple z-band in base_link
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(ranged);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(static_cast<float>(min_height_), static_cast<float>(max_height_));
      pass.filter(*band);
    }

    // Publish filtered cloud (still in base_link)
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*band, out);
    out.header = cloud_base.header;  // frame_id = base_link, stamp preserved
    pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<zed_interfaces::msg::PlaneStamped>::SharedPtr ground_plane_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string input_topic_, output_topic_;
  double min_range_, max_range_, min_height_, max_height_, voxel_leaf_;

  std::mutex plane_mtx_;
  PlaneCache latest_plane_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleFilterNode>());
  rclcpp::shutdown();
  return 0;
}

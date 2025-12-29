#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <cmath>
#include <string>
#include <filesystem>

class DEMCostmapConverter : public rclcpp::Node
{
public:
    DEMCostmapConverter() : Node("dem_costmap_converter")
    {
        // Declare parameters
        this->declare_parameter<std::string>("dem_file_path", "");
        this->declare_parameter<double>("map_resolution", 1.0);  // meters per pixel
        this->declare_parameter<double>("max_passable_slope_degrees", 15.0);
        this->declare_parameter<std::string>("output_frame", "map");
        this->declare_parameter<double>("origin_x", 0.0);
        this->declare_parameter<double>("origin_y", 0.0);
        
        // Get parameters
        std::string dem_file_path = this->get_parameter("dem_file_path").as_string();
        map_resolution_ = this->get_parameter("map_resolution").as_double();
        max_passable_slope_degrees_ = this->get_parameter("max_passable_slope_degrees").as_double();
        output_frame_ = this->get_parameter("output_frame").as_string();
        origin_x_ = this->get_parameter("origin_x").as_double();
        origin_y_ = this->get_parameter("origin_y").as_double();
        
        // Create publisher with transient_local QoS for static maps
        costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "map", rclcpp::QoS(1).transient_local());
        
        RCLCPP_INFO(this->get_logger(), "DEM Costmap Converter initialized");
        RCLCPP_INFO(this->get_logger(), "DEM file: %s", dem_file_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Resolution: %.2f m/pixel", map_resolution_);
        RCLCPP_INFO(this->get_logger(), "Max passable slope: %.1f degrees", max_passable_slope_degrees_);
        
        // Load and process DEM
        costmap_ready_ = false;
        if (!dem_file_path.empty()) {
            loadAndProcessDEM(dem_file_path);
        } else {
            RCLCPP_ERROR(this->get_logger(), "No DEM file path provided. Use param 'dem_file_path'");
            RCLCPP_ERROR(this->get_logger(), "Node will not publish costmap without valid DEM file");
        }
        
        // Publish at 1Hz
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DEMCostmapConverter::publishCostmap, this));
    }

private:
    void loadAndProcessDEM(const std::string& file_path)
    {
        RCLCPP_INFO(this->get_logger(), "Loading DEM from: %s", file_path.c_str());
        
        // Check if file exists
        if (!std::filesystem::exists(file_path)) {
            RCLCPP_ERROR(this->get_logger(), "DEM file does not exist: %s", file_path.c_str());
            return;
        }
        
        // Load DEM using OpenCV (supports TIFF)
        cv::Mat dem = cv::imread(file_path, cv::IMREAD_ANYDEPTH | cv::IMREAD_GRAYSCALE);
        
        if (dem.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load DEM file: %s", file_path.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "DEM loaded successfully: %dx%d pixels", dem.cols, dem.rows);
        
        // Convert to float for calculations
        cv::Mat dem_float;
        dem.convertTo(dem_float, CV_32F);
        
        // Calculate slope map
        cv::Mat slope_map = calculateSlope(dem_float);
        
        // Convert slope to costmap
        costmap_msg_ = createCostmapFromSlope(slope_map, dem.cols, dem.rows);
        costmap_ready_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Costmap generated successfully");
    }
    
    cv::Mat calculateSlope(const cv::Mat& dem)
    {
        RCLCPP_INFO(this->get_logger(), "Calculating slope map...");
        
        cv::Mat grad_x, grad_y;
        cv::Mat slope_radians = cv::Mat::zeros(dem.size(), CV_32F);
        
        // Calculate gradients using Sobel operator
        cv::Sobel(dem, grad_x, CV_32F, 1, 0, 3);
        cv::Sobel(dem, grad_y, CV_32F, 0, 1, 3);
        
        // Scale gradients by resolution to get proper slope
        grad_x = grad_x / (8.0 * map_resolution_);  // Sobel scale factor is 8
        grad_y = grad_y / (8.0 * map_resolution_);
        
        // Calculate slope magnitude
        for (int i = 0; i < dem.rows; ++i) {
            for (int j = 0; j < dem.cols; ++j) {
                float dx = grad_x.at<float>(i, j);
                float dy = grad_y.at<float>(i, j);
                float slope_rad = std::atan(std::sqrt(dx*dx + dy*dy));
                slope_radians.at<float>(i, j) = slope_rad;
            }
        }
        
        // Convert to degrees
        cv::Mat slope_degrees;
        slope_radians *= (180.0 / M_PI);
        slope_degrees = slope_radians;
        
        return slope_degrees;
    }
    
    nav_msgs::msg::OccupancyGrid createCostmapFromSlope(const cv::Mat& slope_map, int width, int height)
    {
        RCLCPP_INFO(this->get_logger(), "Converting slope to costmap...");
        
        nav_msgs::msg::OccupancyGrid costmap;
        costmap.header.frame_id = output_frame_;
        costmap.header.stamp = this->get_clock()->now();
        
        // Set map info
        costmap.info.resolution = map_resolution_;
        costmap.info.width = width;
        costmap.info.height = height;
        costmap.info.origin.position.x = origin_x_;
        costmap.info.origin.position.y = origin_y_;
        costmap.info.origin.position.z = 0.0;
        costmap.info.origin.orientation.w = 1.0;
        
        // Calculate and log costmap bounds
        double costmap_max_x = origin_x_ + (width * map_resolution_);
        double costmap_max_y = origin_y_ + (height * map_resolution_);
        
        RCLCPP_INFO(this->get_logger(), "Costmap bounds:");
        RCLCPP_INFO(this->get_logger(), "  Origin: (%.2f, %.2f)", origin_x_, origin_y_);
        RCLCPP_INFO(this->get_logger(), "  Size: %dx%d pixels", width, height);
        RCLCPP_INFO(this->get_logger(), "  Resolution: %.2f m/pixel", map_resolution_);
        RCLCPP_INFO(this->get_logger(), "  Bounds: X[%.2f, %.2f], Y[%.2f, %.2f]", 
                   origin_x_, costmap_max_x, origin_y_, costmap_max_y);
        RCLCPP_INFO(this->get_logger(), "  Total coverage: %.2f x %.2f meters", 
                   width * map_resolution_, height * map_resolution_);
        
        // Resize data array
        costmap.data.resize(width * height);
        
        // Convert slope to cost values following nav2 costmap2d architecture
        int lethal_count = 0;
        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                float slope_deg = slope_map.at<float>(i, j);
                int cost = slopeToCost(slope_deg);
                
                // OpenCV uses (row, col) but OccupancyGrid uses (x, y)
                // Need to flip Y coordinate for ROS convention
                int ros_i = height - 1 - i;
                costmap.data[ros_i * width + j] = cost;
                
                if (cost >= 99) lethal_count++;
            }
        }
        
        RCLCPP_INFO(this->get_logger(), 
                   "Costmap conversion complete. Lethal/high-cost cells: %d/%d (%.1f%%)",
                   lethal_count, width * height, 
                   100.0 * lethal_count / (width * height));
        
        return costmap;
    }
    
    int slopeToCost(float slope_degrees)
    {
        // Nav2 costmap2d cost values:
        // 0: Free space
        // 1-252: Scaled costs
        // 253: Possibly circumscribed 
        // 254: Unknown
        // 255: Lethal obstacle (impassable)
        
        if (slope_degrees >= max_passable_slope_degrees_) {
            return 255;  // Lethal - impassable
        } else if (slope_degrees >= 10.0) {
            // High cost: linear scale from 150-252 for 10-15 degrees
            float ratio = (slope_degrees - 10.0) / (max_passable_slope_degrees_ - 10.0);
            return static_cast<int>(150 + ratio * 102);
        } else if (slope_degrees >= 5.0) {
            // Medium cost: linear scale from 50-149 for 5-10 degrees  
            float ratio = (slope_degrees - 5.0) / 5.0;
            return static_cast<int>(50 + ratio * 99);
        } else {
            // Low cost: linear scale from 0-49 for 0-5 degrees
            float ratio = slope_degrees / 5.0;
            return static_cast<int>(ratio * 49);
        }
    }

    
    void publishCostmap()
    {
        if (costmap_ready_) {
            costmap_msg_.header.stamp = this->get_clock()->now();
            costmap_publisher_->publish(costmap_msg_);
            RCLCPP_INFO(this->get_logger(), "Published costmap: %dx%d, resolution: %.2f, origin: (%.2f, %.2f)", 
                        costmap_msg_.info.width, costmap_msg_.info.height, costmap_msg_.info.resolution,
                        costmap_msg_.info.origin.position.x, costmap_msg_.info.origin.position.y);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Costmap not ready, skipping publish");
        }
    }

    // Member variables
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid costmap_msg_;
    bool costmap_ready_;
    
    double map_resolution_;
    double max_passable_slope_degrees_;
    std::string output_frame_;
    double origin_x_;
    double origin_y_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DEMCostmapConverter>());
    rclcpp::shutdown();
    return 0;
}
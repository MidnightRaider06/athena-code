#ifndef LOCALIZER_STATE_ESTIMATOR_H_
#define LOCALIZER_STATE_ESTIMATOR_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include <deque>
#include <mutex>
#include <atomic>
#include <optional>

namespace localizer {

using gtsam::symbol_shorthand::X;  // Pose
using gtsam::symbol_shorthand::V;  // Velocity
using gtsam::symbol_shorthand::B;  // Bias

// ============================================================================
// Enums and Structs
// ============================================================================

enum class InitState {
    WAITING_FOR_IMU,
    WAITING_FOR_POSITION,
    RUNNING
};

struct EstimatedState {
    rclcpp::Time timestamp;
    gtsam::NavState nav_state;
    gtsam::imuBias::ConstantBias imu_bias;
    Eigen::Matrix<double, 15, 15> covariance;
    gtsam::Vector3 body_rate;  // Angular velocity in body frame (gyro - bias)

    nav_msgs::msg::Odometry to_odometry(
        const std::string& frame_id,
        const std::string& child_frame_id) const;
};

struct ImuMeasurement {
    rclcpp::Time timestamp;
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;
    
    ImuMeasurement(const rclcpp::Time& t, const Eigen::Vector3d& a, const Eigen::Vector3d& g)
        : timestamp(t), accel(a), gyro(g) {}
};

struct GraphStateEntry {
    uint64_t key_index;
    rclcpp::Time timestamp;
    gtsam::NavState nav_state;
    gtsam::imuBias::ConstantBias bias;
    gtsam::Vector3 latest_gyro;  // Most recent gyro measurement at this state
};

struct PendingGnss {
    gtsam::Point3 position;
    double sigma = 0.0;
    rclcpp::Time timestamp;
    bool valid = false;
    
    void reset() {
        valid = false;
        sigma = 0.0;
    }
};

struct PendingOdom {
    gtsam::Pose3 delta;
    std::optional<gtsam::Pose3> prev_pose;
    bool valid = false;
    
    void reset() {
        valid = false;
        delta = gtsam::Pose3();
        prev_pose.reset();
    }
};

struct FrameParams {
    std::string map_frame = "map";
    std::string odom_frame = "odom";
    std::string base_frame = "base_link";
    std::string imu_frame = "imu_link";
    std::string gnss_frame = "gnss_link";
    std::string tf_prefix;
};

struct CachedFrameIds {
    std::string map;
    std::string odom;
    std::string base;
};

struct StateEstimatorParams {
    // IMU noise (continuous-time spectral densities)
    double accel_noise_sigma = 0.1;
    double gyro_noise_sigma = 0.01;
    double accel_bias_rw_sigma = 0.001;
    double gyro_bias_rw_sigma = 0.0001;
    
    // Sensor noise
    double gnss_position_sigma = 1.0;
    double gnss_altitude_sigma = 0.15;
    double odom_position_sigma = 0.1;
    double odom_rotation_sigma = 0.05;
    
    // Graph management
    size_t min_imu_samples_for_init = 100;
    double state_creation_rate = 10.0;
    size_t max_graph_states = 100;
    double gnss_max_age = 0.5;
    
    // Frames
    FrameParams frames;
    
    // ISAM2
    double isam2_relinearize_threshold = 0.1;
    int isam2_relinearize_skip = 1;
    
    // IMU integration
    double integration_covariance = 1e-8;
    double bias_acc_omega_int = 1e-5;
    
    // Initialization
    double initial_velocity_sigma = 0.1;
    double initial_rotation_sigma = 0.1;
    double initial_accel_bias_sigma = 0.1;
    double initial_gyro_bias_sigma = 0.01;
    double gnss_fix_quality_multiplier = 2.0;
    
    // GNSS outlier rejection
    bool use_robust_gnss_noise = true;
    double gnss_huber_k = 1.345;
    
    // Buffer and timing
    size_t imu_buffer_max_size = 1000;
    double tf_publish_rate = 50.0;

    // IMU filtering (EMA)
    bool enable_imu_filter = true;
    double imu_filter_alpha = 0.3;
};

// ============================================================================
// StateEstimator Class
// ============================================================================

class StateEstimator : public rclcpp::Node {
public:
    StateEstimator();
    ~StateEstimator();
    
    void reset();
    InitState get_init_state() const;
    std::optional<EstimatedState> get_latest_state() const;
    
    // For testing
    rclcpp::Time get_imu_buffer_start_time() const;
    rclcpp::Time get_imu_buffer_end_time() const;

private:
    // Parameter handling
    void declare_parameters();
    void validate_parameters();
    void init_imu_params();
    void init_noise_models();
    void cache_frame_ids();
    
    // Sensor callbacks
    void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg);
    void gnss_callback(sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
    
    // Initialization
    bool initialize_graph(const gtsam::Point3& position, const rclcpp::Time& timestamp);
    gtsam::Rot3 estimate_initial_orientation_locked() const;
    
    // State creation
    void create_new_state(const rclcpp::Time& timestamp);
    void marginalize_old_states();
    
    // IMU integration
    void integrate_imu_measurements_locked(const rclcpp::Time& from_time, const rclcpp::Time& to_time);
    void reset_preintegration(const gtsam::imuBias::ConstantBias& bias);
    void prune_imu_buffer_locked(const rclcpp::Time& before);
    
    // Coordinate transforms
    gtsam::Point3 lla_to_enu(double lat, double lon, double alt);
    void set_enu_origin(double lat, double lon, double alt);
    
    // TF handling
    void lookup_sensor_transforms();
    void publish_state();
    void publish_transforms();

    // Parameters
    StateEstimatorParams params_;
    
    // ISAM2 optimizer
    std::unique_ptr<gtsam::ISAM2> isam_;
    
    // IMU preintegration
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imu_params_;
    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> preintegrated_imu_;
    
    // Pre-computed noise models
    gtsam::SharedNoiseModel initial_pose_noise_;
    gtsam::SharedNoiseModel initial_vel_noise_;
    gtsam::SharedNoiseModel initial_bias_noise_;
    gtsam::SharedNoiseModel odom_noise_;
    
    // State tracking
    std::vector<GraphStateEntry> graph_states_;
    uint64_t current_key_index_;
    uint64_t oldest_key_index_;
    std::atomic<InitState> init_state_;
    
    // Timing
    double min_state_dt_;
    rclcpp::Time last_state_time_;
    
    // IMU buffer
    std::deque<ImuMeasurement> imu_buffer_;
    
    // Pending measurements
    PendingGnss pending_gnss_;
    PendingOdom pending_odom_;
    
    // ENU origin
    bool enu_origin_set_;
    double origin_lat_;
    double origin_lon_;
    double origin_alt_;
    
    // Cached sensor transforms
    std::optional<gtsam::Pose3> imu_to_base_;
    std::optional<gtsam::Point3> gnss_to_base_;
    bool imu_extrinsic_set_;  // Flag to track if IMU extrinsic is initialized

    // EMA filter state (for accel x, y, z and gyro x, y, z)
    std::optional<Eigen::Vector3d> filtered_accel_;
    std::optional<Eigen::Vector3d> filtered_gyro_;
    bool filter_initialized_;

    // Cached frame IDs
    CachedFrameIds cached_frames_;
    
    // Synchronization - mutable for const methods
    mutable std::mutex state_mutex_;
    mutable std::mutex imu_mutex_;
    
    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr filtered_imu_pub_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
};

}  // namespace localizer

#endif  // LOCALIZER_STATE_ESTIMATOR_H_
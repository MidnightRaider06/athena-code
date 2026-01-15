# State Estimator Configuration

Configuration reference for the GTSAM-based state estimator.

## Topics

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `imu_topic` | string | `/imu` | IMU measurements input |
| `gnss_topic` | string | `/gps/fix` | GNSS position fixes |
| `odom_topic` | string | `/odom/ground_truth` | Wheel odometry input |
| `output_odom_topic` | string | `/localization/odom` | Fused state output |

## Frame IDs

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tf_prefix` | string | `""` | Namespace prefix for all frames |
| `map_frame` | string | `map` | Global reference frame |
| `odom_frame` | string | `odom` | Odometry frame (drift-free subset) |
| `base_frame` | string | `base_footprint` | Robot body frame |
| `imu_frame` | string | `imu_link` | IMU sensor frame |
| `gnss_frame` | string | `gnss_link` | GNSS antenna frame |

## Rates

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `publish_rate` | double | 50.0 | Odometry message publish rate (Hz) |
| `tf_publish_rate` | double | 50.0 | Transform broadcast rate (Hz) |
| `state_creation_rate` | double | 50.0 | Graph state creation rate (Hz). Higher values = denser graph, more compute |

## IMU Noise Model

These are continuous-time noise densities for the IMU preintegration.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `accel_noise_sigma` | double | 0.1 | Accelerometer white noise (m/s²/√Hz) |
| `gyro_noise_sigma` | double | 0.01 | Gyroscope white noise (rad/s/√Hz) |
| `accel_bias_rw_sigma` | double | 1e-4 | Accel bias random walk (m/s³/√Hz) |
| `gyro_bias_rw_sigma` | double | 1e-5 | Gyro bias random walk (rad/s²/√Hz) |

**Note:** Z-axis accelerometer uncertainty is automatically scaled 10x higher due to gravity dominance and poor vertical observability.

### IMU Filtering (Optional)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enable_imu_filter` | bool | false | Apply exponential moving average filter to raw IMU |
| `imu_filter_alpha` | double | 0.3 | EMA smoothing factor (0=no update, 1=no smoothing) |

## GNSS Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `gnss_position_sigma` | double | 1.0 | Base horizontal position uncertainty (m) |
| `gnss_altitude_sigma` | double | 0.15 | Vertical position uncertainty (m) |
| `gnss_max_age` | double | 1.0 | Maximum message age before rejection (s) |
| `gnss_fix_quality_multiplier` | double | 2.0 | Uncertainty multiplier for standard fix vs RTK |

### Robust Estimation

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_robust_gnss_noise` | bool | true | Enable Huber M-estimator for outlier rejection |
| `gnss_huber_k` | double | 1.345 | Huber threshold (lower = more aggressive rejection) |

## Odometry Parameters

Noise model for relative odometry measurements.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `odom_position_sigma` | double | 0.05 | Translation uncertainty (m) |
| `odom_rotation_sigma` | double | 0.05 | Rotation uncertainty (rad) |

## ISAM2 Optimizer

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `isam2_relinearize_threshold` | double | 0.1 | Relinearization threshold (lower = more accurate but slower) |
| `isam2_relinearize_skip` | int | 1 | Relinearize every N updates (1 = every time) |

## Graph Management

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_graph_states` | size_t | 100 | Maximum states in graph before marginalization |
| `imu_buffer_max_size` | size_t | 1000 | Maximum IMU measurements to buffer |

**Duration:** At 50 Hz state creation, 100 states = 2 seconds of history.

## Integration Parameters

Internal preintegration tuning - usually don't touch these.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `integration_covariance` | double | 1e-8 | IMU integration uncertainty scalar |
| `bias_acc_omega_int` | double | 1e-5 | Bias coupling during integration |

## Initialization

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_imu_samples_for_init` | size_t | 100 | IMU samples needed for gravity alignment |
| `initial_velocity_sigma` | double | 0.1 | Initial velocity prior uncertainty (m/s) |
| `initial_rotation_sigma` | double | 0.1 | Initial orientation prior (rad) |
| `initial_accel_bias_sigma` | double | 0.1 | Initial accelerometer bias prior (m/s²) |
| `initial_gyro_bias_sigma` | double | 0.01 | Initial gyroscope bias prior (rad/s) |

## ENU Origin

Set the local tangent plane origin. If not provided, uses first GNSS fix.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `origin_lat` | double | 0.0 | Latitude (degrees) |
| `origin_lon` | double | 0.0 | Longitude (degrees) |
| `origin_alt` | double | 0.0 | Altitude (meters) |

## Common Tuning Scenarios

### High-Rate IMU with Drift
- Increase `accel_bias_rw_sigma` and `gyro_bias_rw_sigma` to allow bias tracking
- Increase `max_graph_states` to maintain longer optimization window

### Poor GNSS Quality
- Increase `gnss_position_sigma`
- Enable `use_robust_gnss_noise`
- Lower `gnss_huber_k` for aggressive outlier rejection

### Computational Constraints
- Reduce `state_creation_rate`
- Reduce `max_graph_states`
- Increase `isam2_relinearize_skip`

### Initialization Issues
- Increase `min_imu_samples_for_init` for better gravity estimate
- Check IMU is in base_link frame or transform is available

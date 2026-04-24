import math
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import MagneticField, Imu
from msgs.msg import Heading

from ahrs.filters import Mahony


def _q_to_yaw(q: np.ndarray) -> float:
    """Extract ZYX yaw from quaternion [w, x, y, z]."""
    qw, qx, qy, qz = q
    return math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy**2 + qz**2))


def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class MagHeadingNode(Node):
    def __init__(self):
        super().__init__('mag_heading_node')

        self.declare_parameter('mag_topic', '/zed/zed_node/imu/mag')
        self.declare_parameter('imu_topic', '/zed/zed_node/imu/data')
        self.declare_parameter('frame_id',  'base_link')
        self.declare_parameter('hard_iron', [0.0, 0.0, 0.0])
        self.declare_parameter('soft_iron', [1.0, 0.0, 0.0,
                                             0.0, 1.0, 0.0,
                                             0.0, 0.0, 1.0])
        self.declare_parameter('mahony_kp', 2.0)
        self.declare_parameter('mahony_ki', 0.005)
        self.declare_parameter('mag_field_gate', 0.3)

        mag_topic = self.get_parameter('mag_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        hi = self.get_parameter('hard_iron').get_parameter_value().double_array_value
        si = self.get_parameter('soft_iron').get_parameter_value().double_array_value
        self.hard_iron = np.array(hi, dtype=float)
        self.soft_iron = np.array(si, dtype=float).reshape(3, 3)

        kp = self.get_parameter('mahony_kp').value
        ki = self.get_parameter('mahony_ki').value
        self._mag_gate = self.get_parameter('mag_field_gate').value

        self._filter = Mahony(frequency=200.0, k_P=kp, k_I=ki)
        self._q = np.array([1., 0., 0., 0.])

        self._last_mag     = None
        self._last_mag_cov = [0.0] * 9
        self._mag_ref_norm = None

        self._initialized    = False
        self._last_imu_stamp = None

        self.pub = self.create_publisher(Heading, 'heading', 10)
        self.create_subscription(MagneticField, mag_topic, self._mag_cb, 10)
        self.create_subscription(Imu,           imu_topic, self._imu_cb, 10)
        self.get_logger().info(
            f'mag_heading_node (Mahony) | mag={mag_topic}  imu={imu_topic}  '
            f'Kp={kp}  Ki={ki}'
        )


    def _imu_cb(self, msg: Imu):
        acc   = np.array([msg.linear_acceleration.x,
                          msg.linear_acceleration.y,
                          msg.linear_acceleration.z])
        gyr   = np.array([msg.angular_velocity.x,
                          msg.angular_velocity.y,
                          msg.angular_velocity.z])
        stamp = msg.header.stamp

        if not self._initialized:
            if abs(np.linalg.norm(acc) - 9.81) < 1.0:
                self._initialized = True
                self.get_logger().info('Mahony filter initialised.')
            self._last_imu_stamp = stamp
            return

        if self._last_imu_stamp is None:
            self._last_imu_stamp = stamp
            return
        dt = ((stamp.sec  - self._last_imu_stamp.sec)
              + (stamp.nanosec - self._last_imu_stamp.nanosec) * 1e-9)
        self._last_imu_stamp = stamp
        if dt <= 0.0 or dt > 0.5:
            return

        if self._last_mag is not None:
            self._q = self._filter.updateMARG(
                self._q, gyr=gyr, acc=acc, mag=self._last_mag, dt=dt
            )
        else:
            self._q = self._filter.updateIMU(self._q, gyr=gyr, acc=acc, dt=dt)

        self._publish(stamp)

    def _mag_cb(self, msg: MagneticField):
        raw   = np.array([msg.magnetic_field.x,
                          msg.magnetic_field.y,
                          msg.magnetic_field.z])
        m_cal  = self.soft_iron @ (raw - self.hard_iron)
        m_norm = np.linalg.norm(m_cal)
        if m_norm < 1e-9:
            return

        if self._mag_ref_norm is None:
            self._mag_ref_norm = m_norm
            self.get_logger().info(f'Mag reference magnitude: {m_norm:.4f}')

        if abs(m_norm - self._mag_ref_norm) > self._mag_gate * self._mag_ref_norm:
            self.get_logger().warn(
                f'Mag rejected: |m|={m_norm:.4f} deviates '
                f'>{self._mag_gate*100:.0f}% from ref {self._mag_ref_norm:.4f}'
            )
            return

        self._last_mag     = m_cal
        self._last_mag_cov = list(msg.magnetic_field_covariance)

    def _publish(self, stamp):
        yaw = _wrap_pi(_q_to_yaw(self._q))

        heading_acc = 0.0
        if self._last_mag is not None:
            cov  = self._last_mag_cov
            b_xy = math.hypot(self._last_mag[0], self._last_mag[1])
            if b_xy > 1e-9 and cov[0] > 0.0:
                heading_acc = math.sqrt((cov[0] + cov[4]) / 2.0) / b_xy

        compass_bearing_deg = (90.0 - math.degrees(yaw)) % 360.0

        out = Heading()
        out.header.stamp    = stamp
        out.header.frame_id = self.frame_id
        out.heading         = yaw
        out.heading_acc     = heading_acc
        out.compass_bearing = compass_bearing_deg
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MagHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

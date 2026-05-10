#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from msgs.msg import LedStatus

import serial
import serial.tools.list_ports

_SYNC = 0xAA


def _packet(cmd: int, r: int = 0, g: int = 0, b: int = 0, param: int = 0) -> bytes:
    xor = cmd ^ r ^ g ^ b ^ param
    return bytes([_SYNC, cmd, r, g, b, param, xor])


def _find_data_port() -> str | None:
    ports = sorted(
        p.device
        for p in serial.tools.list_ports.comports()
        if "trinkey" in (p.description or "").lower()
        or "trinkey" in (p.product or "").lower()
        or "adafruit" in (p.manufacturer or "").lower()
    )
    return ports[-1] if ports else None


class LedIndicatorNode(Node):
    def __init__(self):
        super().__init__("led_indicator")

        self.declare_parameter("serial_port", "auto")
        self.declare_parameter("baud_rate", 115200)

        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baud_rate").value

        if port == "auto":
            port = _find_data_port()
            if port is None:
                self.get_logger().warn(
                    "Trinkey data port not found — defaulting to /dev/ttyACM1. "
                    "Set 'serial_port' explicitly if wrong."
                )
                port = "/dev/ttyACM1"
            else:
                self.get_logger().info(f"Auto-detected Trinkey data port: {port}")

        self._ser: serial.Serial | None = None
        try:
            self._ser = serial.Serial(port, baud, timeout=1, dsrdtr=False, rtscts=False)
            self.get_logger().info(f"Connected to Pixel Trinkey on {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open {port}: {e}. LED output disabled.")

        reliable_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(LedStatus, "/led_status", self._cb, reliable_qos)

        self._send(_packet(LedStatus.CMD_OFF))

    def _cb(self, msg: LedStatus) -> None:
        self._send(_packet(msg.cmd, msg.r, msg.g, msg.b, msg.param))

    def _send(self, pkt: bytes) -> None:
        if self._ser is None or not self._ser.is_open:
            return
        try:
            self._ser.write(pkt)
        except serial.SerialException as e:
            self.get_logger().warn(f"Serial write failed: {e}", throttle_duration_sec=5.0)

    def destroy_node(self) -> None:
        self._send(_packet(LedStatus.CMD_OFF))
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LedIndicatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

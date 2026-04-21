#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import socket
import struct
import time

CAN_FMT  = "=IH2x8s"
CAN_SIZE = struct.calcsize(CAN_FMT)


CAN_IDS       = [0x145, 0x141, 0x142, 0x144]
CAN_INTERFACE = "can1"
MAX_SPEED_DPS = 500.0
SCALE         = 100.0
TOPIC         = "/rear_ackermann_controller/reference"


SPEED_CONTROL_CMD = 0xA2




class CmdVelToRMD(Node):
   def __init__(self):
       super().__init__("cmd_vel_to_rmd")


       self._sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
       self._sock.bind((CAN_INTERFACE,))
       self.get_logger().info(f"Listening on {TOPIC}, sending to {[hex(i) for i in CAN_IDS]}")


       self.create_subscription(TwistStamped, TOPIC, self._cb, 10)


   def _cb(self, msg: TwistStamped):
       speed_dps  = msg.twist.linear.x * SCALE
       speed_dps  = max(-MAX_SPEED_DPS, min(MAX_SPEED_DPS, speed_dps))
       speed_ctrl = int(speed_dps / 0.01) & 0xFFFFFFFF


       data = bytes([
           SPEED_CONTROL_CMD, 0x00, 0x00, 0x00,
           (speed_ctrl)       & 0xFF,
           (speed_ctrl >> 8)  & 0xFF,
           (speed_ctrl >> 16) & 0xFF,
           (speed_ctrl >> 24) & 0xFF,
       ])


       for can_id in CAN_IDS:
           self._sock.send(struct.pack(CAN_FMT, can_id, 8, data))
           time.sleep(.001)


   def destroy_node(self):
       self._sock.close()
       super().destroy_node()




def main(args=None):
   rclpy.init(args=args)
   node = CmdVelToRMD()
   try:
       rclpy.spin(node)
   except KeyboardInterrupt:
       pass
   finally:
       node.destroy_node()
       rclpy.shutdown()




if __name__ == "__main__":
   main()




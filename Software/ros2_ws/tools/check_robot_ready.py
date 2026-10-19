import sys, os, time
import rclpy
from rclpy.node import Node
from doghead_interfaces.msg import McuState

from rclpy.qos import QoSProfile, ReliabilityPolicy

rclpy.init()
node = rclpy.create_node('robot_ready_checker')

received = False
def cb(msg):
    global received
    print(f"STATE: safety_state={msg.safety_state}, boot_id={msg.boot_id}, q_rad={[round(x,4) for x in msg.measured_position_rad]}")
    received = True

qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
sub = node.create_subscription(McuState, '/doghead/mcu/state', cb, qos)

start = time.time()
while not received and time.time() - start < 3.0:
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()

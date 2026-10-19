import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from doghead_interfaces.msg import McuState

def main():
    rclpy.init()
    node = Node('debug_state_sub')
    qos = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE
    )
    count = 0
    def cb(msg):
        nonlocal count
        count += 1
        print(f"[{count}] McuState seq={msg.state_sequence}, boot_id={hex(msg.boot_id)}, agent_connected={msg.agent_connected}")

    sub = node.create_subscription(McuState, '/doghead/mcu/state', cb, qos)
    print("Listening on /doghead/mcu/state (Best Effort)...")
    
    import time
    t0 = time.time()
    while time.time() - t0 < 3.0 and count < 10:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"Total McuState received in 3s: {count}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

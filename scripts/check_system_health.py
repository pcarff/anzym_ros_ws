#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, JointState
from tf2_msgs.msg import TFMessage
import time

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class SystemHealthCheck(Node):
    def __init__(self):
        super().__init__('system_health_check')
        self.scan_delay = None
        self.has_tf = False
        self.has_joints = False
        self.joint_names = []
        
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_best_effort)
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        self.start_time = time.time()
        # Diagnostic list
        self.timer = self.create_timer(1.0, self.report_status)

    def scan_callback(self, msg):
        current_ros_time = self.get_clock().now()
        scan_time = Time.from_msg(msg.header.stamp)
        self.scan_delay = (current_ros_time - scan_time).nanoseconds / 1e9
        
    def tf_callback(self, msg):
        self.has_tf = True
        
    def joint_callback(self, msg):
        self.has_joints = True
        self.joint_names = msg.name

    def report_status(self):
        print("\n--- System Health Status ---")
        
        # Check Scan Delay
        if self.scan_delay is not None:
            print(f"[SCAN] Latest Scan Delay: {self.scan_delay:.3f}s")
            if abs(self.scan_delay) > 0.2:
                print("       [FAIL] SIGNIFICANT DELAY/OFFSET DETECTED (>200ms)")
            else:
                print("       [PASS] Timestamp Sync OK")
        else:
            print("[SCAN] Waiting for /scan messages... (Lidar might be down)")

        # Check TF
        if self.has_tf:
            print("[TF]   Receiving /tf messages [PASS]")
        else:
            print("[TF]   NO /tf messages received [FAIL]")

        # Check Joints
        if self.has_joints:
            print("[JNT]  Receiving /joint_states [PASS]")
            print(f"       Joints found: {self.joint_names}")
            if 'arm_joint1' in self.joint_names:
                print("       [PASS] Arm Joints Present")
            else:
                print("       [FAIL] Arm Joints MISSING (Driver not publishing?)")
        else:
            print("[JNT]  NO /joint_states messages [FAIL]")
            
        print("----------------------------")

def main(args=None):
    rclpy.init(args=args)
    node = SystemHealthCheck()
    
    print("Collecting data for 5 seconds...")
    try:
        # Spin a bit to collect data
        start = time.time()
        while rclpy.ok() and (time.time() - start) < 5.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            
        node.report_status()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

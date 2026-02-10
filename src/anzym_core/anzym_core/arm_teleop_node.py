#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import math

class ArmTeleopNode(Node):
    def __init__(self):
        super().__init__('arm_teleop_node')
        
        # Mappings - Adjust these if they don't match your controller
        # Default assume Xbox/Logitech layout
        self.declare_parameter('axis_base', 0)          # Left Stick X
        self.declare_parameter('axis_shoulder', 1)      # Left Stick Y
        self.declare_parameter('axis_elbow', 7)         # D-pad Y (Up/Down)
        self.declare_parameter('axis_wrist_pitch', 6)   # D-pad X (Left/Right)
        
        self.declare_parameter('button_wrist_roll_left', 4)  # L1
        self.declare_parameter('axis_wrist_roll_right', 2)   # L2 (Axis)
        
        self.declare_parameter('button_gripper_close', 5)   # R1
        self.declare_parameter('axis_gripper_open', 5)      # R2 (Axis)
        
        self.declare_parameter('button_arm_enable', 5)     # R1 - Deadman switch for arm
        
        self.declare_parameter('button_pose_home', 0)  # A
        self.declare_parameter('button_pose_ready', 1) # B
        self.declare_parameter('button_pose_pick', 2)  # X
        self.declare_parameter('button_pose_drop', 3)  # Y

        # Current joint positions (in radians)
        # J1: Base, J2: Shoulder, J3: Elbow, J4: Wrist Pitch, J5: Wrist Roll, J6: Gripper
        self.joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Poses [J1, J2, J3, J4, J5, J6]
        # 0.0 is center (hardware 90 deg)
        self.poses = {
            'home':  [0.0, -1.2, 1.2, 0.8, 0.0, 0.0],  # Tucked in
            'ready': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],    # Straight up
            'pick':  [0.0, 0.5, -0.6, -0.4, 0.0, 0.6],  # Lowered, gripper open
            'drop':  [1.4, 0.3, -0.3, -0.3, 0.0, -0.6]  # Side, gripper closed
        }

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.arm_pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        
        self.timer = self.create_timer(0.05, self.publish_joints)
        self.get_logger().info("Anzym Arm Teleop Node Started (Hold R1 to move joints)")

    def joy_callback(self, msg):
        # 1. Pose Selection (Right side buttons) - Always active for convenience
        if msg.buttons[self.get_parameter('button_pose_home').value]:
            self.joints = list(self.poses['home'])
            return # Don't process Incremental if pose button is hit
        elif msg.buttons[self.get_parameter('button_pose_ready').value]:
            self.joints = list(self.poses['ready'])
            return
        elif msg.buttons[self.get_parameter('button_pose_pick').value]:
            self.joints = list(self.poses['pick'])
            return
        elif msg.buttons[self.get_parameter('button_pose_drop').value]:
            self.joints = list(self.poses['drop'])
            return
        
        # 2. Manual Incremental Control - ONLY if Enable Button (R1) is held
        if not msg.buttons[self.get_parameter('button_arm_enable').value]:
            return

        step = 0.03 # Radians per tick
        
        # Base & Shoulder (Left Stick)
        self.joints[0] += msg.axes[self.get_parameter('axis_base').value] * step
        self.joints[1] += msg.axes[self.get_parameter('axis_shoulder').value] * step
        
        # Elbow (D-pad Up/Down)
        if len(msg.axes) > 7:
            self.joints[2] += msg.axes[self.get_parameter('axis_elbow').value] * step
            # Wrist Pitch (D-pad Left/Right)
            self.joints[3] += msg.axes[self.get_parameter('axis_wrist_pitch').value] * step

        # Wrist Roll (L1 / L2)
        if msg.buttons[self.get_parameter('button_wrist_roll_left').value]:
            self.joints[4] += step
        if msg.axes[self.get_parameter('axis_wrist_roll_right').value] < 0: # Trigger pressed
            self.joints[4] -= step

        # Gripper (R1 / R2)
        if msg.buttons[self.get_parameter('button_gripper_close').value]:
            self.joints[5] += step # Close
        if msg.axes[self.get_parameter('axis_gripper_open').value] < 0: # Trigger pressed
            self.joints[5] -= step # Open

        # Simple Safety Clamping
        for i in range(5):
            self.joints[i] = max(min(self.joints[i], 1.57), -1.57) # +/- 90 degrees
        self.joints[5] = max(min(self.joints[5], 1.57), -1.57) # Gripper limits

    def publish_joints(self):
        msg = Float64MultiArray()
        msg.data = self.joints
        self.arm_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

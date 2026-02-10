#!/usr/bin/env python3
"""
Arm Teleop Node for Anzym X3 Plus Robot
========================================
Controller Layout:

  RIGHT SIDE BUTTONS (Poses):
    Y  = Straight Up     (all joints 90 deg)
    X  = Init Pose       (folded start position)
    A  = Pose A           (placeholder - customize)
    B  = Pose B           (placeholder - customize)

  TRIGGERS (Gripper):
    LT (Left Trigger)   = Open gripper
    RT (Right Trigger)   = Close gripper

  BUMPERS (Wrist Roll / J5):
    LB (Left Bumper)   = Rotate wrist left  (J5 -)
    RB (Right Bumper)  = Rotate wrist right (J5 +)

  D-PAD (Framework - placeholder):
    Up/Down/Left/Right = (not yet assigned)

  JOYSTICKS = Used for robot wheel movement via teleop_twist_joy
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import math

# ── Radian helpers ──────────────────────────────────────────
def deg2rad(hw_deg, center=90.0):
    """Convert a hardware servo angle to the radian value expected by driver_node."""
    return (hw_deg - center) * (math.pi / 180.0)


class ArmTeleopNode(Node):
    def __init__(self):
        super().__init__('arm_teleop_node')

        # ── Joystick index parameters ──────────────────────────
        # Pose buttons – verified by user testing
        self.declare_parameter('button_Y', 4)       # Y = Straight Up
        self.declare_parameter('button_X', 3)       # X = Init Pose
        self.declare_parameter('button_A', 0)       # A = Pose A (placeholder)
        self.declare_parameter('button_B', 1)       # B = Pose B (placeholder)

        # Bumpers (wrist roll) – verified by user testing
        self.declare_parameter('button_LB', 6)      # Left Bumper
        self.declare_parameter('button_RB', 7)      # Right Bumper

        # D-Pad axes
        self.declare_parameter('axis_dpad_x', 6)
        self.declare_parameter('axis_dpad_y', 7)

        # Speed
        self.declare_parameter('step_size', 0.03)

        # ── Preset Poses ───────────────────────────────────────
        # [J1=Base, J2=Shoulder, J3=Elbow, J4=WristPitch, J5=WristRoll, J6=Gripper]
        self.poses = {
            'straight_up': [
                deg2rad(90, 90),   # J1
                deg2rad(90, 90),   # J2
                deg2rad(90, 90),   # J3
                deg2rad(90, 90),   # J4
                deg2rad(90, 90),   # J5
                deg2rad(90, 0),    # J6 (gripper mid)
            ],
            'init': [
                deg2rad(90, 90),   # J1
                deg2rad(135, 90),  # J2
                deg2rad(0, 90),    # J3
                deg2rad(0, 90),    # J4
                deg2rad(90, 90),   # J5
                deg2rad(180, 0),   # J6 (gripper closed)
            ],
            'pose_a': [
                deg2rad(90, 90),
                deg2rad(90, 90),
                deg2rad(45, 90),
                deg2rad(45, 90),
                deg2rad(90, 90),
                deg2rad(90, 0),
            ],
            'pose_b': [
                deg2rad(90, 90),
                deg2rad(45, 90),
                deg2rad(135, 90),
                deg2rad(90, 90),
                deg2rad(90, 90),
                deg2rad(90, 0),
            ],
        }

        # Current joint state (start at init pose)
        self.joints = list(self.poses['init'])

        # Joint limits (radians) – maps to hardware 0-180 deg
        self.j_min = [deg2rad(0, 90)] * 5 + [deg2rad(0, 0)]
        self.j_max = [deg2rad(180, 90)] * 5 + [deg2rad(180, 0)]

        # Track whether we've logged the axis mapping
        self._axis_map_logged = False
        self._last_axes = None

        # ── ROS interfaces ─────────────────────────────────────
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.arm_pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        self.timer = self.create_timer(0.05, self.publish_joints)

        self.get_logger().info('Arm Teleop Ready  |  Y=Up  X=Init  LT/RT=Gripper  LB/RB=WristRoll')

    # ── Helpers ────────────────────────────────────────────────
    def clamp_joints(self):
        for i in range(6):
            self.joints[i] = max(self.j_min[i], min(self.j_max[i], self.joints[i]))

    def _detect_trigger(self, msg):
        """Detect trigger pull from Joy axes.
        
        Different controllers report triggers differently:
        - Xbox/Logitech: axes rest at 1.0, go to -1.0 when pressed
        - Some: axes rest at 0.0, go to 1.0 when pressed  
        - Some: triggers are buttons, not axes
        
        We detect by checking each axis for significant deviation from rest.
        Returns (lt_pressed, rt_pressed) booleans.
        """
        lt_pressed = False
        rt_pressed = False
        
        # Try button-based triggers first (more reliable)
        # On many controllers, LT/RT are also mapped as buttons 6/7 or similar
        # But they overlap with Back/Start. So we use axis-based.
        
        # For axis-based triggers:
        # We look for axes that have moved significantly from their neighbors.
        # Common layouts:
        #   axes[2] = LT or RStickX
        #   axes[5] = RT or RStickY
        # Since axes[2] is used by teleop_twist_joy for yaw, it's the right stick.
        # Let's check axes 4 and 5 first (common for standalone triggers).
        
        num_axes = len(msg.axes)
        
        # Log the full axis map once so user can identify
        if not self._axis_map_logged:
            self.get_logger().info(f'Controller has {num_axes} axes and {len(msg.buttons)} buttons')
            self.get_logger().info(f'Axes values at rest: {[round(a, 2) for a in msg.axes]}')
            self._axis_map_logged = True
        
        # Strategy: Check axes 4 and 5 with threshold.
        # If axis rests at 1.0 → pressed when < 0.0
        # If axis rests at 0.0 → we can't use it (conflicts with sticks)
        # If axis rests at -1.0 → pressed when > 0.0
        
        # Axes 4/5 as triggers (common mapping for standalone trigger axes)
        if num_axes > 4:
            val = msg.axes[4]
            if val < -0.5:   # Pulled down significantly
                lt_pressed = True
            elif val > 0.8:  # Resting at ~1.0 means NOT pressed
                lt_pressed = False
                
        if num_axes > 5:
            val = msg.axes[5]
            if val < -0.5:
                rt_pressed = True
            elif val > 0.8:
                rt_pressed = False
        
        return lt_pressed, rt_pressed

    # ── Joy callback ───────────────────────────────────────────
    def joy_callback(self, msg):
        step = self.get_parameter('step_size').value

        # ── 1.  POSE BUTTONS (right side) ──────────────────────
        if msg.buttons[self.get_parameter('button_Y').value]:
            self.joints = list(self.poses['straight_up'])
            self.get_logger().info('Pose → Straight Up')
            return

        if msg.buttons[self.get_parameter('button_X').value]:
            self.joints = list(self.poses['init'])
            self.get_logger().info('Pose → Init')
            return

        if msg.buttons[self.get_parameter('button_A').value]:
            self.joints = list(self.poses['pose_a'])
            self.get_logger().info('Pose → A')
            return

        if msg.buttons[self.get_parameter('button_B').value]:
            self.joints = list(self.poses['pose_b'])
            self.get_logger().info('Pose → B')
            return

        # ── 2.  GRIPPER  (LT = open,  RT = close) ─────────────
        lt_pressed, rt_pressed = self._detect_trigger(msg)

        if lt_pressed:
            self.joints[5] -= step  # Open gripper
        if rt_pressed:
            self.joints[5] += step  # Close gripper

        # ── 3.  WRIST ROLL  (LB = left,  RB = right) ──────────
        lb_idx = self.get_parameter('button_LB').value
        rb_idx = self.get_parameter('button_RB').value
        wrist_step = 0.15  # Bigger step for wrist roll (~8.6 deg)

        if lb_idx < len(msg.buttons) and msg.buttons[lb_idx]:
            self.joints[4] -= wrist_step  # Rotate wrist left
            self.get_logger().info(f'Wrist roll LEFT → J5={self.joints[4]:.2f}')
        if rb_idx < len(msg.buttons) and msg.buttons[rb_idx]:
            self.joints[4] += wrist_step  # Rotate wrist right
            self.get_logger().info(f'Wrist roll RIGHT → J5={self.joints[4]:.2f}')

        # ── 4.  D-PAD  (framework – uncomment and assign) ─────
        dpad_x_idx = self.get_parameter('axis_dpad_x').value
        dpad_y_idx = self.get_parameter('axis_dpad_y').value

        if dpad_x_idx < len(msg.axes):
            dpad_x = msg.axes[dpad_x_idx]
            if dpad_x > 0.5:
                pass  # TODO: D-Pad Left action
            elif dpad_x < -0.5:
                pass  # TODO: D-Pad Right action

        if dpad_y_idx < len(msg.axes):
            dpad_y = msg.axes[dpad_y_idx]
            if dpad_y > 0.5:
                pass  # TODO: D-Pad Up action
            elif dpad_y < -0.5:
                pass  # TODO: D-Pad Down action

        # ── Clamp all joints ───────────────────────────────────
        self.clamp_joints()

    # ── Publish at a steady rate ───────────────────────────────
    def publish_joints(self):
        msg = Float64MultiArray()
        msg.data = list(self.joints)
        self.arm_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

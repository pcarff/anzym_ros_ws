#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math
import time
from std_msgs.msg import Float64MultiArray
from anzym_core.Rosmaster_Lib import Rosmaster

class RosmasterDriver(Node):
    def __init__(self):
        super().__init__('rosmaster_driver')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('car_type', 2)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('init_pose', [90.0, 135.0, 0.0, 0.0, 90.0, 180.0]) # Default up/center pose

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.car_type = self.get_parameter('car_type').get_parameter_value().integer_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

        self.get_logger().info(f'Connecting to Rosmaster on {self.port}...')
        
        try:
            self.bot = Rosmaster(car_type=self.car_type, com=self.port, debug=False)
            self.bot.create_receive_threading() 
            self.bot.set_car_type(self.car_type)
            self.bot.set_auto_report_state(True, forever=False) 
            self.bot.set_beep(50)

            # Initialize Arm Pose
            init_pose = self.get_parameter('init_pose').get_parameter_value().double_array_value
            if len(init_pose) == 6:
                init_angles = [int(x) for x in init_pose]
                self.get_logger().info(f'Setting initial arm pose: {init_angles}')
                self.bot.set_uart_servo_angle_array(init_angles)
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            return

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Subscriber
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.arm_sub = self.create_subscription(Float64MultiArray, 'joint_commands', self.arm_cmd_callback, 10)
        
        # Timer for Odometry (20Hz)
        self.create_timer(0.05, self.update_odometry)

        # State vars
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        
        # Last known joint positions for fallback
        self.last_joint_position = None

        self.get_logger().info('Rosmaster Driver Ready with Odometry')

    def arm_cmd_callback(self, msg):
        # Expecting 6 values in Radians: [Joint1, Joint2, Joint3, Joint4, Joint5, Gripper]
        if len(msg.data) != 6:
            self.get_logger().warn(f"Received arm command with {len(msg.data)} values, expected 6")
            return
            
        def rad2deg(rad, center=90.0):
            return int((rad * (180.0 / math.pi)) + center)
            
        try:
            # Map Radians to Degrees for Hardware
            # Check limits (hardware limits are usually 0-180 or 0-270)
            
            # Joint 1 (Base): Center 90
            angle_1 = rad2deg(msg.data[0], 90.0)
            
            # Joint 2: Center 90
            angle_2 = rad2deg(msg.data[1], 90.0)
            
            # Joint 3: Center 90
            angle_3 = rad2deg(msg.data[2], 90.0)
            
            # Joint 4: Center 90
            angle_4 = rad2deg(msg.data[3], 90.0)
            
            # Joint 5: Center 90
            angle_5 = rad2deg(msg.data[4], 90.0)
            
            # Joint 6 (Gripper): Center 0 in logic? No, hardware is 30-180 usually.
            # Let's assume input is 0.0 to 1.57 (0 to 90 deg) or similar.
            # If msg.data[5] is already radians from 0->Close, we convert.
            # Gripper is tricky. Let's assume passed value is rads from 0.
            angle_6 = rad2deg(msg.data[5], 0.0) # Hardware likely expects 30-180
            
            # Clamp values to safe ranges (0-180 is safe for standard servos)
            angles = [angle_1, angle_2, angle_3, angle_4, angle_5, angle_6]
            angles = [max(0, min(180, a)) for a in angles]
            
            # Send to robot
            # set_uart_servo_angle_array(self, angle_s=[90, 90, 90, 90, 90, 180], run_time=500)
            self.bot.set_uart_servo_angle_array(angles, run_time=500)
            
        except Exception as e:
            self.get_logger().error(f"Error setting arm angles: {e}")

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        vz = msg.angular.z
        
        # Manual Mecanum Mixing (Standard X3/X3Plus Map)
        # Verify M1=FL, M2=RL, M3=FR, M4=RR wiring assumption
        # X3 Plus often uses:
        # V1 = Vx - Vy - Vz * (Ax + Ay)
        # V2 = Vx + Vy - Vz * (Ax + Ay)
        # V3 = Vx - Vy + Vz * (Ax + Ay)
        # V4 = Vx + Vy + Vz * (Ax + Ay)
        
        # Simplified ratio mixing (works for most generic mecanum):
        v1 = vx - vy - vz  # FL
        v2 = vx + vy - vz  # RL
        v3 = vx + vy + vz  # FR (Different from standard sometimes?)
        v4 = vx - vy + vz  # RR
        
        # Note: Yahboom might be M1=FL, M2=RL, M3=RR, M4=FR ??
        # Standard:
        # M1 FL: +x +y +z | (vx - vy - vz)
        # M2 RL: +x -y +z | (vx + vy - vz) 
        # M3 FR: +x -y -z | (vx + vy + vz) NO -> (vx + vy + vz) is usually FR
        # M4 RR: +x +y -z | (vx - vy + vz)
        
        # Let's trust the mixing that WORKED in Step 617:
        # v_fl = (vx - vy - vz)
        # v_rl = (vx + vy - vz)
        # v_fr = (vx + vy + vz)
        # v_rr = (vx - vy + vz)
        # set_motor(v_fl, v_rl, v_fr, v_rr)
        
        # Scale to PWM (approx 100 max)
        # A speed of 1.0 m/s is roughly 100 PWM? No.
        # Let's apply a gain. 
        GAIN = 100.0 # Arbitrary gain. 1.0 m/s -> 100 PWM
        
        m1 = int(v1 * GAIN)
        m2 = int(v2 * GAIN)
        m3 = int(v3 * GAIN)
        m4 = int(v4 * GAIN)
        
        self.bot.set_motor(m1, m2, m3, m4)

    def update_odometry(self):
        # Read velocities from MCU (parsed by Rosmaster_Lib from serial report)
        # The Lib stores them in PRIVATE variables: self.__vx, etc.
        # But looking at Rosmaster_Lib code, they are NOT exposed via getters!?
        # Wait, I need to check Rosmaster_Lib again.
        # It defines __vx etc. There are NO getter methods in the snippet I saw!
        
        # Hack: Access private vars directly via name mangling or add getters?
        # Python name mangling: _Rosmaster__vx
        
        try:
            vx = -float(self.bot._Rosmaster__vx)
            vy = -float(self.bot._Rosmaster__vy)
            vth = -float(self.bot._Rosmaster__vz)
        except AttributeError:
            # If mangling fails or vars init differently
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Compute odometry in a "global" frame (simple integration)
        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Quaternion
        q = self.euler_to_quaternion(0, 0, self.th)

        # Publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

        # Publish Odom
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        
        self.odom_pub.publish(odom)

        # Publish Joint States (Read from Servos)
        
        # Read angles from hardware (returns list of 6 angles in degrees)
        # Note: get_uart_servo_angle_array might return None or -1 on error
        try:
            angles = self.bot.get_uart_servo_angle_array()
            if angles and len(angles) == 6 and angles[0] != -1:
                # Convert degrees to radians (Assuming 90 deg is center/0.0 for joints 1-5)
                # Joint 6 (Gripper) might be different
                # Structure: [S1, S2, S3, S4, S5, S6]
                
                # Helper to convert: (angle - center) * (PI / 180)
                def deg2rad(deg, center=90.0):
                    return (deg - center) * (math.pi / 180.0)

                # Joint 1: Base Rotation (90 is center)
                j1 = deg2rad(angles[0], 90.0)
                
                # Joint 2: Shoulder (90 is center?)
                j2 = deg2rad(angles[1], 90.0)
                
                # Joint 3: Elbow (90 is center?)
                j3 = deg2rad(angles[2], 90.0)
                
                # Joint 4: Wrist Pitch (90 is center?)
                j4 = deg2rad(angles[3], 90.0)
                
                # Joint 5: Wrist Roll (90 is center?)
                j5 = deg2rad(angles[4], 90.0) # OR maybe 180 is center? 
                
                # Joint 6: Gripper
                # Gripper usually 30-180? Let's just pass raw rads relative to 0 for now
                # Or maybe normalize it. 
                j6 = deg2rad(angles[5], 0.0) 
                
                self.last_joint_position = [float(j1), float(j2), float(j3), float(j4), float(j5), float(j6)]
            
            # If we haven't read successfully yet, don't publish anything
            if self.last_joint_position is None:
                return

            joint_state = JointState()
            joint_state.header.stamp = current_time.to_msg()
            joint_state.name = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5', 'grip_joint']
            joint_state.position = self.last_joint_position
            self.joint_pub.publish(joint_state)

        except Exception as e:
            # Only log if we have previously succeeded, to avoid spamming on startup if off
            if self.last_joint_position is not None:
                self.get_logger().warn(f"Failed to read servos: {e}")
                # Publish last known
                joint_state = JointState()
                joint_state.header.stamp = current_time.to_msg()
                joint_state.name = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5', 'grip_joint']
                joint_state.position = self.last_joint_position
                self.joint_pub.publish(joint_state)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = RosmasterDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

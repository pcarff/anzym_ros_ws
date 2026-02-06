#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from anzym_core.Rosmaster_Lib import Rosmaster
import math

class RosmasterDriver(Node):
    def __init__(self):
        super().__init__('rosmaster_driver')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('car_type', 1)
        
        port = self.get_parameter('port').get_parameter_value().string_value
        car_type = self.get_parameter('car_type').get_parameter_value().integer_value
        
        self.get_logger().info(f'Connecting to Rosmaster on {port}...')
        
        try:
            # Disable debug=True for production
            self.bot = Rosmaster(car_type=car_type, com=port, debug=False)
            # IMPORTANT: Start the receive thread to handle incoming data/heartbeats
            self.bot.create_receive_threading()
            
            # Explicitly set car type (Required for V3.3.1 hardware)
            # 1 = X3, 2 = X3 Plus
            self.get_logger().info(f'Sending Set Car Type: {car_type}')
            self.bot.set_car_type(car_type)
            
            # Short blip to confirm connection
            self.bot.set_beep(50)
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Rosmaster: {e}')
            return

        # Subscriber
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.get_logger().info('Rosmaster Driver Ready')

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        vz = msg.angular.z
        
        # Log at DEBUG unless analyzing motion issues
        self.get_logger().debug(f'Vel: x={vx:.2f}, y={vy:.2f}, z={vz:.2f}')
        
        # Use custom mixer to solve strafing issue
        # Standard Mecanum Formula (roughly):
        # FL = x - y - z
        # FR = x + y + z
        # RL = x + y - z
        # RR = x - y + z
        
        # Rosmaster Hardware Mapping (Estimated based on user report):
        # M1=RR, M2=RL, M3=FL, M4=FR
        # NOTE: We need to tune this based on observation.
        
        speed_scale = 100.0 # Map -1.0..1.0 to -100..100 PWM
        
        # Based on "Left Stick Left" -> Fronts Back, Rears Fwd
        # We want: 
        # FL (M2?): Back 
        # FR (M4?): Fwd
        # RL (M1?): Fwd
        # RR (M3?): Back
        
        # Let's try standard mixing and map to the 4 registers
        # We assume standard Yahboom mapping:
        # M1: Rear Left
        # M2: Front Left
        # M3: Front Right
        # M4: Rear Right
        
        # Standard Mecanum Kinematics
        # V_fl = Vx - Vy - Vz
        # V_fr = Vx + Vy + Vz
        # V_rl = Vx + Vy - Vz
        # V_rr = Vx - Vy + Vz

        v_fl = (vx - vy - vz) * speed_scale
        v_fr = (vx + vy + vz) * speed_scale
        v_rl = (vx + vy - vz) * speed_scale
        v_rr = (vx - vy + vz) * speed_scale
        
        # Clamp to [-100, 100]
        v_fl = max(-100, min(100, v_fl))
        v_fr = max(-100, min(100, v_fr))
        v_rl = max(-100, min(100, v_rl))
        v_rr = max(-100, min(100, v_rr))
        
        # Send to robot using set_motor(m1, m2, m3, m4)
        
        # -------------------------------------------------------------------------
        #  Anzym Hardware Specific Mapping (NON-STANDARD)
        # -------------------------------------------------------------------------
        # Diagnosis confirmed the following physical motor wiring on this specific unit:
        # M1 = Front Left
        # M2 = Rear Left
        # M3 = Rear Right (Standard would be Front Right)
        # M4 = Front Right (Standard would be Rear Right)
        
        # Apply Mapping for Anzym:
        # M1 <-- v_fl
        # M2 <-- v_rl
        # M3 <-- v_rr (Swapped)
        # M4 <-- v_fr (Swapped)
        self.bot.set_motor(int(v_fl), int(v_rl), int(v_rr), int(v_fr))

        # -------------------------------------------------------------------------
        #  Standard Yahboom X3 Configuration
        # -------------------------------------------------------------------------
        # If you are using a standard X3, your wiring likely matches the label:
        # M1=FL, M2=RL, M3=FR, M4=RR.
        # Uncomment the line below for standard wiring:
        # self.bot.set_motor(int(v_fl), int(v_rl), int(v_fr), int(v_rr))
        # -------------------------------------------------------------------------
        
        # self.bot.set_car_motion(vx, vy, vz)

def main(args=None):
    rclpy.init(args=args)
    node = RosmasterDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

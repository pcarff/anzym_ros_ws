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
        
        # Standard Firmware Kinematics
        # Since physical wiring is now standard (M1=FL, M2=RL, M3=FR, M4=RR),
        # we act like a standard X3 and let the MCU handle mixing.
        self.bot.set_car_motion(vx, vy, vz)

def main(args=None):
    rclpy.init(args=args)
    node = RosmasterDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

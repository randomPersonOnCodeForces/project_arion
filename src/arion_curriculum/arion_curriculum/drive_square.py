import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class DriveSquare(Node):
    def __init__(self):
        super().__init__('drive_square')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # State: 0=Forward, 1=Turn, 2=Stop
        self.state = 0
        self.start_time = time.time()
        self.count = 0  # Count how many sides we've done
        
        self.get_logger().info('Driving in a square...')

    def timer_callback(self):
        msg = Twist()
        current_time = time.time()
        elapsed = current_time - self.start_time

        # CONFIGURATION
        drive_time = 2.0   # Drive forward for 2 seconds
        turn_time = 2.2    # Turn for 2.2 seconds (Adjust this to tune 90 degrees!)
        speed = 0.2
        turn_speed = 0.7

        if self.state == 0: # STATE: FORWARD
            msg.linear.x = speed
            msg.angular.z = 0.0
            if elapsed > drive_time:
                self.state = 1
                self.start_time = time.time()
                self.get_logger().info('Turning...')

        elif self.state == 1: # STATE: TURN
            msg.linear.x = 0.0
            msg.angular.z = turn_speed
            if elapsed > turn_time:
                self.count += 1
                if self.count >= 4:
                    self.state = 2 # Finished 4 sides
                else:
                    self.state = 0 # Back to forward
                self.start_time = time.time()
                self.get_logger().info('Forward...')

        elif self.state == 2: # STATE: STOP
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            # Optional: Shutdown after finishing
            # rclpy.shutdown()

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DriveSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
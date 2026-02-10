import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DriveSquare(Node):
    def __init__(self):
        super().__init__('drive_square')
        
        # Publisher to send velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer to run the control loop every 0.1 seconds
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # State variables for the student to use
        self.state = 0  # 0=Forward, 1=Turn
        self.start_time = time.time()
        
        self.get_logger().info('Drive Square Node Started! Students: Implement your logic below.')

    def timer_callback(self):
        msg = Twist()
        current_time = time.time()
        elapsed = current_time - self.start_time

        # =============================================================
        # STUDENT TODO:
        # Implement a state machine to drive in a square.
        # Hint:
        #   - Drive forward (linear.x > 0) for 2 seconds
        #   - Turn left (angular.z > 0) for 1 second (approx 90 degrees)
        #   - Repeat 4 times
        # =============================================================
        
        # Example logic (Students delete this and write their own):
        if self.state == 0:
            msg.linear.x = 0.0 # Change this!
            # if elapsed > 2.0: switch to turn state
            
        elif self.state == 1:
            msg.angular.z = 0.0 # Change this!
            # if elapsed > 1.0: switch back to forward state

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DriveSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

class StopAtWall(Node):
    def __init__(self):
        super().__init__('stop_at_wall')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # "Best Effort" QoS is required to receive data from Gazebo sensors
        qos_policy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            qos_policy)
            
        self.get_logger().info('Stop At Wall Node Started! Waiting for Lidar data...')

    def scan_callback(self, msg):
        cmd = Twist()
        
        # msg.ranges is a list of 360 distance measurements (floats)
        # Index 0 is directly ahead.
        front_dist = msg.ranges[0]

        # =============================================================
        # STUDENT TODO:
        # 1. Check if 'front_dist' is less than 0.5 meters.
        # 2. If yes, stop the robot (linear.x = 0.0).
        # 3. If no, drive forward safely (linear.x = 0.2).
        # =============================================================

        # Debug print (Optional)
        # self.get_logger().info(f'Distance: {front_dist:.2f}m')

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = StopAtWall()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
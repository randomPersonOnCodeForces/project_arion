import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

class StopAtWall(Node):
    def __init__(self):
        super().__init__('stop_at_wall')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # QoS MUST match Gazebo (Best Effort) or you won't get data
        qos_policy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            qos_policy)
            
        self.get_logger().info('Safety Stop Node Active. Move an object in front of the robot!')

    def scan_callback(self, msg):
        cmd = Twist()
        
        # msg.ranges[0] is the distance directly ahead (0 degrees)
        # We replace 'inf' (infinity) with a large number to avoid errors
        front_dist = msg.ranges[0]
        if float(front_dist) == float('inf'):
            front_dist = 999.0

        if front_dist < 0.5:
            # DANGER ZONE
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info(f'STOPPING! Obstacle at {front_dist:.2f}m')
        else:
            # SAFE ZONE
            cmd.linear.x = 0.2  # Drive forward slowly
            cmd.angular.z = 0.0
            
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = StopAtWall()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
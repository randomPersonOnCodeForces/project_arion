import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

class StopAtWall(Node):
    def __init__(self):
        super().__init__('stop_at_wall')

        # --- PART 2: PARAMETERIZATION ---
        # We declare a parameter named 'stop_distance' with a default of 0.5m
        self.declare_parameter('stop_distance', 0.5)
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        qos_policy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            qos_policy)

        # --- PART 1: SAFETY WATCHDOG ---
        # Track when we last heard from the Lidar
        self.last_scan_time = self.get_clock().now()
        # Check for safety every 0.1 seconds (10 Hz)
        self.timer = self.create_timer(0.1, self.watchdog_callback)
        self.is_safe_to_drive = False
            
        self.get_logger().info('Safety Stop Node Active. Monitoring Lidar...')

    def watchdog_callback(self):
        # Calculate time since last scan
        time_diff = self.get_clock().now() - self.last_scan_time
        seconds_since_scan = time_diff.nanoseconds / 1e9

        # If data is older than 1.0 second, TRIGGER KILL SWITCH
        if seconds_since_scan > 1.0:
            self.get_logger().warn(f'LIDAR TIMEOUT! Last signal {seconds_since_scan:.1f}s ago. STOPPING.', throttle_duration_sec=2)
            self.stop_robot()
            self.is_safe_to_drive = False

    def scan_callback(self, msg):
        # Reset the watchdog timer
        self.last_scan_time = self.get_clock().now()
        self.is_safe_to_drive = True

        # Get the parameter value dynamically (allows live tuning)
        stop_threshold = self.get_parameter('stop_distance').get_parameter_value().double_value

        cmd = Twist()
        
        # Handle 'inf' values
        front_dist = msg.ranges[0]
        if float(front_dist) == float('inf'):
            front_dist = 999.0

        if front_dist < stop_threshold:
            self.get_logger().info(f'OBSTACLE DETECTED at {front_dist:.2f}m! Stopping.')
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0
            
        self.publisher_.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
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
#!/usr/bin/env python3

import heapq
import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver')

        self.declare_parameter('algorithm', 'astar')
        self.algorithm = self.get_parameter('algorithm').get_parameter_value().string_value.lower()
        if self.algorithm not in ('astar', 'bfs'):
            self.get_logger().warn(
                f"Unsupported algorithm '{self.algorithm}', defaulting to 'astar'."
            )
            self.algorithm = 'astar'

        # 5x5 occupancy grid matching simple_floor.sdf (0: free, 1: wall)
        self.maze = [
            [0, 0, 1, 0, 0],
            [1, 0, 1, 0, 1],
            [1, 0, 0, 0, 1],
            [1, 1, 1, 0, 1],
            [1, 1, 1, 0, 0],
        ]
        self.rows = len(self.maze)
        self.cols = len(self.maze[0])
        self.cell_size = 0.5

        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.min_scan_range = float('inf')

        self.waypoints = []
        self.current_waypoint_index = 0

        self.safety_stop_distance = 0.14
        self.waypoint_tolerance = 0.08
        self.k_linear = 0.6
        self.k_angular = 1.8
        self.max_linear = 0.22
        self.max_angular = 1.6

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 20)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 20)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f"maze_solver started with algorithm='{self.algorithm}'.")

    def odom_callback(self, msg: Odometry) -> None:
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def scan_callback(self, msg: LaserScan) -> None:
        valid = [r for r in msg.ranges if math.isfinite(r)]
        self.min_scan_range = min(valid) if valid else float('inf')

    def goal_callback(self, msg: PoseStamped) -> None:
        if self.current_x is None or self.current_y is None:
            self.get_logger().warn('Ignoring goal: no odometry received yet.')
            return

        start = self.world_to_grid(self.current_x, self.current_y)
        goal = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)

        if start is None:
            self.get_logger().warn('Current robot pose is outside maze grid.')
            return
        if goal is None:
            self.get_logger().warn('Goal pose is outside maze grid.')
            return
        if self.is_wall(start):
            self.get_logger().warn('Current robot cell maps to a wall; cannot plan.')
            return
        if self.is_wall(goal):
            self.get_logger().warn('Goal cell is a wall; choose a free cell.')
            return

        if self.algorithm == 'bfs':
            path = self.plan_bfs(start, goal)
        else:
            path = self.plan_astar(start, goal)

        if not path:
            self.get_logger().warn(f'No path found from {start} to {goal}.')
            self.waypoints = []
            self.current_waypoint_index = 0
            self.stop_robot()
            return

        self.waypoints = [self.grid_to_world(cell[0], cell[1]) for cell in path][1:]
        self.current_waypoint_index = 0
        self.get_logger().info(
            f'Received goal -> start={start}, goal={goal}, path_len={len(path)} cells.'
        )

    def control_loop(self) -> None:
        if self.min_scan_range < self.safety_stop_distance:
            self.stop_robot()
            return

        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return

        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            self.stop_robot()
            return

        target_x, target_y = self.waypoints[self.current_waypoint_index]
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.hypot(dx, dy)

        if distance < self.waypoint_tolerance:
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info('Goal reached.')
                self.stop_robot()
            return

        target_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_heading - self.current_yaw)

        cmd = Twist()
        cmd.angular.z = self.clamp(self.k_angular * heading_error, -self.max_angular, self.max_angular)

        if abs(heading_error) < 0.5:
            cmd.linear.x = self.clamp(self.k_linear * distance, 0.0, self.max_linear)
        else:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)

    def world_to_grid(self, x: float, y: float):
        col = int(round(x / self.cell_size))
        row = int(round(y / self.cell_size))
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return (row, col)
        return None

    def grid_to_world(self, row: int, col: int):
        return (col * self.cell_size, row * self.cell_size)

    def is_wall(self, cell) -> bool:
        row, col = cell
        return self.maze[row][col] == 1

    def neighbors(self, cell):
        row, col = cell
        candidates = [
            (row - 1, col),
            (row + 1, col),
            (row, col - 1),
            (row, col + 1),
        ]
        result = []
        for r, c in candidates:
            if 0 <= r < self.rows and 0 <= c < self.cols and self.maze[r][c] == 0:
                result.append((r, c))
        return result

    def plan_bfs(self, start, goal):
        queue = deque([start])
        came_from = {start: None}

        while queue:
            current = queue.popleft()
            if current == goal:
                return self.reconstruct_path(came_from, goal)

            for nxt in self.neighbors(current):
                if nxt not in came_from:
                    came_from[nxt] = current
                    queue.append(nxt)

        return []

    def plan_astar(self, start, goal):
        open_heap = []
        heapq.heappush(open_heap, (0.0, start))

        came_from = {start: None}
        g_score = {start: 0.0}

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == goal:
                return self.reconstruct_path(came_from, goal)

            for nxt in self.neighbors(current):
                tentative_g = g_score[current] + 1.0
                if nxt not in g_score or tentative_g < g_score[nxt]:
                    g_score[nxt] = tentative_g
                    f_score = tentative_g + self.manhattan(nxt, goal)
                    heapq.heappush(open_heap, (f_score, nxt))
                    came_from[nxt] = current

        return []

    def reconstruct_path(self, came_from, goal):
        path = [goal]
        current = goal
        while came_from[current] is not None:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    @staticmethod
    def manhattan(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    def stop_robot(self) -> None:
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MazeSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

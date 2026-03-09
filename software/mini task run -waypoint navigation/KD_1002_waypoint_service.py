#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from waypoint_navigation.srv import GetWaypoints


class WayPoints(Node):

    def __init__(self):
        super().__init__('waypoints_service')
        self.srv = self.create_service(GetWaypoints, 'waypoints', self.waypoint_callback)

        # Task 2B Waypoints (No pesticide station 2)
        self.waypoints = [
            # Ground Station
            [-6.90, 0.09, 32.16],
            [-7.00, 0.00, 30.22],

            # Pesticide Station 1
            [-7.64, 3.06, 30.22],
            [-8.22, 6.02, 30.22],
            [-9.11, 9.27, 31.27],
            [-9.07, 9.23, 32.57],

            # Block 1
            [-9.11, 9.27, 31.27],
            [-5.98, 8.81, 31.27],
            [-3.26, 8.41, 29.88],
            [0.87, 8.18, 29.05],
            [3.93, 7.35, 29.05],
            [6.60, 6.62, 30.20],

            # Block 2
            [6.74, 3.32, 30.64],
            [6.87, -0.19, 30.09],
            [7.01, -3.59, 31.53],
            [7.15, -7.00, 31.98],

            # Plant Hover & Spray Positions
            [-5.35, 5.15, 30.20],
            [-5.35, 5.15, 29.40],
            [0.00, 5.00, 30.00],
            [0.00, 5.00, 29.20],
            [5.30, 5.10, 30.10],
            [5.30, 5.10, 29.30],

            # Return to Ground
            [0.00, 0.00, 30.00],
            [-6.90, 0.09, 32.16],
        ]

    def waypoint_callback(self, request, response):
        if request.get_waypoints:
            response.waypoints.poses = [Pose() for _ in range(len(self.waypoints))]
            for i, point in enumerate(self.waypoints):
                response.waypoints.poses[i].position.x = point[0]
                response.waypoints.poses[i].position.y = point[1]
                response.waypoints.poses[i].position.z = point[2]
            self.get_logger().info(f"Sent {len(self.waypoints)} waypoints for Task 2B (No pesticide 2)")
        else:
            self.get_logger().info("Waypoint request rejected")
        return response


def main():
    rclpy.init()
    node = WayPoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


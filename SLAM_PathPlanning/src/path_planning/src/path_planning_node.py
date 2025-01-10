#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from sensor_msgs.msg import Range
from sensor_msgs.msg import PointCloud2

#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from sensor_msgs.msg import Range
from sensor_msgs.msg import PointCloud2
import math


class DynamicPathPlanningNode:
    def __init__(self):
        rospy.init_node('dynamic_path_planning_node')

        # Publishers
        self.waypoint_pub = rospy.Publisher('/path_planning/waypoints', Point, queue_size=10)

        # Subscriptions
        rospy.Subscriber('/dynamic_map', PointCloud2, self.update_map_callback)
        rospy.Subscriber('/target_position', PoseStamped, self.target_callback)
        rospy.Subscriber('/range_front', Range, self.front_sensor_callback)
        rospy.Subscriber('/range_left', Range, self.left_sensor_callback)
        rospy.Subscriber('/range_right', Range, self.right_sensor_callback)

        # Pathfinding variables
        self.grid_size = 40  # 40x40 grid
        self.cell_size = 0.2  # Each grid cell is 0.2 meters
        self.grid = np.zeros((self.grid_size, self.grid_size))  # Initial empty grid
            # Static target position (9 meters away)
        static_target_x = 4.0  # Target at 9 meters
        static_target_y = 0.0  # Aligned with the robot's y-axis

        self.target_position = (
            int(static_target_x / self.cell_size),
            int(static_target_y / self.cell_size)
        )
        # Dynamic variables
        #self.target_position = None  # Dynamic target position (x, y)
        self.current_position = (self.grid_size // 2, self.grid_size // 2)
        self.obstacle_safety_margin = 1.0  # Safety margin in meters

        rospy.loginfo("DynamicPathPlanningNode initialized successfully.")

    def update_map_callback(self, msg):
        rospy.loginfo("Dynamic map callback triggered.")
        pass  # Process PointCloud2 data to update self.grid (not implemented in this example)

    def target_callback(self, msg):
        try:
            # Convert target position to grid coordinates
            target_x = int(msg.pose.position.x / self.cell_size)
            target_y = int(msg.pose.position.y / self.cell_size)

            # Calculate distance from robot's current position
            distance_to_target = math.sqrt(
                (target_x - self.current_position[0])**2 + (target_y - self.current_position[1])**2
            )

            # Set target only if it is within the robot's range and grid bounds
            if distance_to_target < (self.grid_size * self.cell_size / 2):
                self.target_position = (target_x, target_y)
                rospy.loginfo("Target position set to: {}".format(self.target_position))
            else:
                rospy.logwarn("Target position out of reachable range. Ignoring.")
                self.target_position = None
        except Exception as e:
            rospy.logerr("Error in target_callback: {}".format(e))

    def front_sensor_callback(self, msg):
        self.update_obstacle_from_sensor(msg.range, 0)  # Angle 0 degrees for front

    def left_sensor_callback(self, msg):
        self.update_obstacle_from_sensor(msg.range, 90)  # Angle 90 degrees for left

    def right_sensor_callback(self, msg):
        self.update_obstacle_from_sensor(msg.range, -90)  # Angle -90 degrees for right

    def update_obstacle_from_sensor(self, range_data, angle_deg):
        if range_data < 0.05 or range_data > 2.0:
            rospy.loginfo("Range data out of bounds: {}".format(range_data))
            return

        try:
            angle_rad = np.radians(angle_deg)
            dx = range_data * np.cos(angle_rad)
            dy = range_data * np.sin(angle_rad)

            obstacle_x = int((self.grid_size // 2) + dx / self.cell_size)
            obstacle_y = int((self.grid_size // 2) + dy / self.cell_size)

            self.update_grid_obstacle(obstacle_x, obstacle_y)
            self.add_safety_margin(obstacle_x, obstacle_y)
        except Exception as e:
            rospy.logerr("Error in update_obstacle_from_sensor: {}".format(e))

    def add_safety_margin(self, x, y):
        safety_cells = int(self.obstacle_safety_margin / self.cell_size)
        for i in range(-safety_cells, safety_cells + 1):
            for j in range(-safety_cells, safety_cells + 1):
                nx, ny = x + i, y + j
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    self.grid[nx, ny] = 1

    def update_grid_obstacle(self, x, y):
        if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
            self.grid[x, y] = 1
            rospy.loginfo("Obstacle added at: ({}, {})".format(x, y))
        else:
            rospy.logwarn("Obstacle position out of bounds: ({}, {})".format(x, y))

    def a_star(self, start, goal):
        try:
            open_set = {start}
            came_from = {}
            g_score = {cell: float('inf') for cell in np.ndindex(self.grid.shape)}
            g_score[start] = 0
            f_score = {cell: float('inf') for cell in np.ndindex(self.grid.shape)}
            f_score[start] = self.heuristic(start, goal)

            while open_set:
                current = min(open_set, key=lambda cell: f_score[cell])
                if current == goal:
                    return self.reconstruct_path(came_from, current)

                open_set.remove(current)
                for neighbor in self.get_neighbors(current):
                    tentative_g_score = g_score[current] + 1
                    if tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                        if neighbor not in open_set:
                            open_set.add(neighbor)

            rospy.logwarn("No path found to goal.")
            return []
        except Exception as e:
            rospy.logerr("Error in A* algorithm: {}".format(e))
            return []

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def get_neighbors(self, cell):
        neighbors = [(cell[0] + dx, cell[1] + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]]
        return [n for n in neighbors if 0 <= n[0] < self.grid_size and 0 <= n[1] < self.grid_size and self.grid[n] == 0]

    def control_loop(self):
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            target = self.target_position
            if self.target_position:
                target = self.target_position
            else:
                rospy.logwarn("No valid target position. Waiting for updates.")
                rate.sleep()
                continue

            if not (0 <= target[0] < self.grid_size and 0 <= target[1] < self.grid_size):
                rospy.logwarn("Adjusted target out of bounds. Skipping this iteration.")
                rate.sleep()
                continue

            path = self.a_star(self.current_position, target)
            if path:
                rospy.loginfo("Path found: {}".format(path))
                for waypoint in path:
                    point = Point()
                    point.x, point.y = waypoint[0] * self.cell_size, waypoint[1] * self.cell_size
                    self.waypoint_pub.publish(point)
                    self.current_position = waypoint
            else:
                rospy.logwarn("No valid path to target.")
            rate.sleep()


if __name__ == '__main__':
    try:
        planner = DynamicPathPlanningNode()
        planner.control_loop()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception caught.")

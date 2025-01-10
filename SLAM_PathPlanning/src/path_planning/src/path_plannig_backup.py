#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from sensor_msgs.msg import Range
from sensor_msgs.msg import PointCloud2


class PathPlanningNode:
    def __init__(self):
        rospy.init_node('path_planning_node')

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

        # Ensure fallback target is within bounds
        self.fallback_target = (
            self.grid_size // 2,
            min(self.grid_size - 1, self.grid_size // 2 + int(9 / self.cell_size))  # Clamp to max index
        )
        self.target_position = None  # Dynamic target position
        self.obstacle_safety_margin = 1.0  # Meters

        rospy.loginfo("PathPlanningNode initialized successfully.")
        rospy.loginfo("Fallback target initialized at: {}".format(self.fallback_target))

    def update_map_callback(self, msg):
        rospy.loginfo("Dynamic map callback triggered.")
        # Process PointCloud2 data and update self.grid (not implemented in this code)
        pass

    def target_callback(self, msg):
        try:
            self.target_position = (int(msg.pose.position.x / self.cell_size),
                                    int(msg.pose.position.y / self.cell_size))
            rospy.loginfo("Dynamic target position updated to: {}".format(self.target_position))
        except Exception as e:
            rospy.logerr("Error in target_callback: {}".format(e))

    def front_sensor_callback(self, msg):
        try:
            grid_pos = (self.grid_size // 2, self.grid_size // 2 + int(msg.range / self.cell_size))
            self.update_grid_obstacle(*grid_pos)
            rospy.loginfo("Front sensor updated obstacle at: {}".format(grid_pos))
        except Exception as e:
            rospy.logerr("Error in front_sensor_callback: {}".format(e))

    def left_sensor_callback(self, msg):
        try:
            grid_pos = (self.grid_size // 2 - int(msg.range / self.cell_size), self.grid_size // 2)
            self.update_grid_obstacle(*grid_pos)
            rospy.loginfo("Left sensor updated obstacle at: {}".format(grid_pos))
        except Exception as e:
            rospy.logerr("Error in left_sensor_callback: {}".format(e))

    def right_sensor_callback(self, msg):
        try:
            grid_pos = (self.grid_size // 2 + int(msg.range / self.cell_size), self.grid_size // 2)
            self.update_grid_obstacle(*grid_pos)
            rospy.loginfo("Right sensor updated obstacle at: {}".format(grid_pos))
        except Exception as e:
            rospy.logerr("Error in right_sensor_callback: {}".format(e))

    def update_grid_obstacle(self, x, y):
        try:
            if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                self.grid[x, y] = 1  # Mark as occupied
                rospy.loginfo("Obstacle updated on grid at: ({}, {})".format(x, y))
            else:
                rospy.logwarn("Obstacle position out of bounds: ({}, {})".format(x, y))
        except Exception as e:
            rospy.logerr("Error in update_grid_obstacle: {}".format(e))

    def a_star(self, start, goal):
        rospy.loginfo("Starting A* from {} to {}.".format(start, goal))
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
                    rospy.loginfo("Path to goal found.")
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

            rospy.logwarn("No path found to the goal.")
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
        rospy.loginfo("Reconstructed path: {}".format(path))
        return path

    def get_neighbors(self, cell):
        neighbors = [(cell[0] + dx, cell[1] + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]]
        return [n for n in neighbors if 0 <= n[0] < self.grid_size and 0 <= n[1] < self.grid_size and self.grid[n] == 0]

    def control_loop(self):
        rate = rospy.Rate(1)  # 1 Hz
        start = (self.grid_size // 2, self.grid_size // 2)

        while not rospy.is_shutdown():
            try:
                # Ensure start and target are within bounds
                if not (0 <= start[0] < self.grid_size and 0 <= start[1] < self.grid_size):
                    rospy.logwarn("Start position out of bounds: {}".format(start))
                    continue

                target = self.target_position if self.target_position else self.fallback_target
                if not (0 <= target[0] < self.grid_size and 0 <= target[1] < self.grid_size):
                    rospy.logwarn("Target position out of bounds: {}".format(target))
                    continue

                rospy.loginfo("Selected target: {}".format(target))
                rospy.loginfo("Grid size: {}x{}".format(self.grid_size, self.grid_size))
                rospy.loginfo("Grid state before pathfinding:\n{}".format(self.grid))

                # Pathfinding
                path = self.a_star(start, target)
                if path:
                    rospy.loginfo("Publishing path with {} waypoints.".format(len(path)))
                    for waypoint in path:
                        point = Point()
                        point.x, point.y = waypoint[0] * self.cell_size, waypoint[1] * self.cell_size
                        self.waypoint_pub.publish(point)
                        rospy.loginfo("Published waypoint: ({}, {})".format(point.x, point.y))
                else:
                    rospy.logwarn("No valid path to target. Robot will remain idle.")
            except Exception as e:
                rospy.logerr("Error in control_loop: {}".format(e))

            rate.sleep()


if __name__ == '__main__':
    try:
        planner = PathPlanningNode()
        planner.control_loop()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception caught.")

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from pcl_helper import convert_to_pcl, pcl_to_ros
import numpy as np

class DynamicMapBuilder:
    def __init__(self, window_size=5):
        rospy.init_node("dynamic_map_builder", anonymous=True)

        # Parameters
        self.window_size = window_size  # Number of point clouds to retain in the sliding window
        self.recent_clouds = []  # Sliding window of recent point clouds
        self.last_received_time = {}  # Track the last received time for each topic

        # Subscribers
        self.subscribers = {
            '/camera1/point_cloud_face': rospy.Subscriber(
                "/camera1/point_cloud_face", PointCloud2, self.pointcloud_callback, callback_args='/camera1/point_cloud_face'
            ),
            '/camera3/point_cloud_left': rospy.Subscriber(
                "/camera3/point_cloud_left", PointCloud2, self.pointcloud_callback, callback_args='/camera3/point_cloud_left'
            ),
            '/camera4/point_cloud_right': rospy.Subscriber(
                "/camera4/point_cloud_right", PointCloud2, self.pointcloud_callback, callback_args='/camera4/point_cloud_right'
            ),
        }

        # Publisher
        self.map_pub = rospy.Publisher("/dynamic_map", PointCloud2, queue_size=10)

        # Publish map periodically
        rospy.Timer(rospy.Duration(1.0), self.publish_map)

    def pointcloud_callback(self, msg, topic_name):
        rospy.loginfo("Received point cloud from {}".format(topic_name))

        # Track the last received time for each topic
        self.last_received_time[topic_name] = rospy.Time.now()

        # Convert PointCloud2 to points
        incoming_points = convert_to_pcl(msg)
        rospy.loginfo("Converted PointCloud2 to {} points.".format(len(incoming_points)))

        # Filter incoming points
        filtered_cloud = self.filter_cloud(incoming_points)

        # Add filtered points to the sliding window
        self.recent_clouds.append(filtered_cloud)
        if len(self.recent_clouds) > self.window_size:
            self.recent_clouds.pop(0)

    def filter_cloud(self, points, leaf_size=0.05):
        """Downsample the point cloud using a simple voxel grid filter."""
        if not points:
            rospy.logwarn("No points to filter.")
            return []

        points_np = np.array(points, dtype=np.float32)
        voxel_indices = (points_np / leaf_size).astype(np.int32)
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
        filtered_points = points_np[unique_indices]
        return filtered_points.tolist()

    def publish_map(self, event):
        # Warn if any topic is missing data
        for topic, last_time in self.last_received_time.items():
            if rospy.Time.now() - last_time > rospy.Duration(2):  # 2 seconds threshold
                rospy.logwarn("No recent data from {}".format(topic))

        # Merge recent clouds into a single list of points
        accumulated_points = []
        for points in self.recent_clouds:
            accumulated_points.extend(points)

        # Convert the accumulated points to a ROS PointCloud2 message and publish
        if accumulated_points:
            ros_cloud = pcl_to_ros(accumulated_points, frame_id="map")
            self.map_pub.publish(ros_cloud)
            rospy.loginfo("Published dynamic map with {} points".format(len(accumulated_points)))
        else:
            rospy.logwarn("No points to publish in the dynamic map.")

if __name__ == "__main__":
    try:
        map_builder = DynamicMapBuilder(window_size=5)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

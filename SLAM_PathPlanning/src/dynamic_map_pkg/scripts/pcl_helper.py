import rospy  # Add this line to import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import std_msgs.msg

def convert_to_pcl(ros_cloud):
    """Convert ROS PointCloud2 message to a list of points."""
    points = []
    for p in pc2.read_points(ros_cloud, skip_nans=True):
        points.append([p[0], p[1], p[2]])
    return points

def pcl_to_ros(points, frame_id="map"):
    """Convert a list of points to a ROS PointCloud2 message."""
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return pc2.create_cloud_xyz32(header, points)

#!/usr/bin/env python

import rospy
import socket
import pickle
import struct
from sensor_msgs.msg import PointCloud2, Range

def start_relay():
    rospy.init_node('relay_node', anonymous=True)

    # Create a socket object
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Connect to the Jetson's IP and port
    jetson_ip = '192.168.123.15'  # Replace with your Jetson's IP
    port = 9090  # Choose an available port (make sure it's not in use)
    s.connect((jetson_ip, port))
    rospy.loginfo("Connected to Jetson at {}:{}".format(jetson_ip, port))

    # List of topics to subscribe to and their message types
    topics = [
        ('/camera1/point_cloud_face', PointCloud2),
        ('/camera3/point_cloud_left', PointCloud2),
        ('/camera4/point_cloud_right', PointCloud2),
        ('/range_front', Range),
        ('/range_left', Range),
        ('/range_right', Range),
    ]

    def callback_factory(topic_name):
        def callback(msg):
            try:
                # Log each transmission attempt
                rospy.loginfo("Relaying data from topic: {}".format(topic_name))
                # Prepare the data
                data = {'topic': topic_name, 'msg': msg}
                # Serialize the data
                serialized_data = pickle.dumps(data, protocol=2)
                # Send message length first
                message_size = struct.pack('>I', len(serialized_data))
                s.sendall(message_size + serialized_data)
            except Exception as e:
                rospy.logerr("Error sending data for topic {}: {}".format(topic_name, e))
        return callback

    # Create subscribers for each topic
    for topic_name, msg_type in topics:
        rospy.Subscriber(topic_name, msg_type, callback_factory(topic_name))
        rospy.loginfo("Subscribed to {}".format(topic_name))

    rospy.spin()
    s.close()

if __name__ == '__main__':
    try:
        start_relay()
    except rospy.ROSInterruptException:
        pass


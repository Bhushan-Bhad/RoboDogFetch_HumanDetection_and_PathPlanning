#!/usr/bin/env python

import rospy
import socket
import pickle
import struct
from sensor_msgs.msg import PointCloud2, Range

def receive_data():
    rospy.init_node('receive_node', anonymous=True)

    # Create a socket object
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Bind to the Jetson's IP and port
    jetson_ip = '192.168.123.15'  # Replace with your Jetson's IP
    port = 9090  # Must match the port used in relay_node.py
    s.bind((jetson_ip, port))
    s.listen(1)
    rospy.loginfo("Listening on {}:{}".format(jetson_ip, port))
    conn, addr = s.accept()
    rospy.loginfo("Connection accepted from {}:{}".format(addr[0], addr[1]))

    # Publishers dictionary
    publishers = {}

    while not rospy.is_shutdown():
        # Read message length
        raw_msglen = recvall(conn, 4)
        if not raw_msglen:
            break
        msglen = struct.unpack('>I', raw_msglen)[0]
        # Read the message data
        data = recvall(conn, msglen)
        if not data:
            break
        # Deserialize the data
        received = pickle.loads(data)
        topic_name = received['topic']
        msg = received['msg']

        # Get or create publisher for the topic
        if topic_name not in publishers:
            # Determine message type
            if isinstance(msg, PointCloud2):
                msg_type = PointCloud2
            elif isinstance(msg, Range):
                msg_type = Range
            else:
                rospy.logwarn('Unknown message type received for topic {}'.format(topic_name))
                continue
            publishers[topic_name] = rospy.Publisher(topic_name, msg_type, queue_size=10)
            rospy.loginfo("Created publisher for {}".format(topic_name))

        # Publish the message
        publishers[topic_name].publish(msg)

    conn.close()
    s.close()

def recvall(conn, n):
    """Helper function to receive n bytes or return None if EOF is hit"""
    data = b''
    while len(data) < n:
        packet = conn.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data

if __name__ == '__main__':
    try:
        receive_data()
    except rospy.ROSInterruptException:
        pass


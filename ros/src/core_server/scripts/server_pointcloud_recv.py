#!/usr/bin/env python
import sys
import rospy
from PIL import Image
from core_msgs.msg import *
from StringIO import StringIO
from base64 import standard_b64decode

class ServerPointCloudRecv():
    def __init__(self):
        self.input_ = rospy.get_param('input', '/core_client_pointcloud/compressed')
        self.output_ = rospy.get_param('output', '/core_server_pointcloud/output')
        rospy.Subscriber(self.input_, PointCloud, self.cloud_callback, queue_size=1)
        self.pub_ = rospy.Publisher(self.output_, PointCloud, queue_size=1)

    def cloud_callback(self, msg):
        """ Decode and publish the message """
        decoded = self.decode(msg.data)
        pub_msg = PointCloud()
        pub_msg.header = msg.header
        pub_msg.data = decoded
        self.pub_.publish(pub_msg)

    def decode(self, string):
        """ b64 decode the string, then PNG-decompress """
        decoded = standard_b64decode(string)
        buff = StringIO(decoded)
        i = Image.open(buff)
        return i.tostring() 

if __name__ == "__main__":
    try:
        rospy.init_node('server_pointcloud_recv')
        server_pointcloud_recv = ServerPointCloudRecv()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
import sys
import rospy
from PIL import Image
from json import dumps
from core_msgs.msg import *
from StringIO import StringIO
from math import floor, ceil, sqrt 
from base64 import standard_b64encode 
from websocket import create_connection

class ClientPointCloudXmit():
    def __init__(self):
        self.ws_ = None
        self.input_ = rospy.get_param('input', '/client_pointcloud_processor/output')
        self.topic_ = rospy.get_param('topic', '/core_client_pointcloud/compressed')
    
    def cloud_callback(self, data):
        """ Encode the data and send it out to the server """
        encoded = self.encode(str(data))
        msg = {"data": encoded}
        pub_msg = dumps({"op": "publish", "topic": self.topic_, "msg": msg})
        self.ws_.send(pub_msg)

    def advertise_topic(self):
        """ Advertise the topic to be published on the server """
        msg_type = "core_msgs/PointCloud"
        pub_msg = dumps({"op": "advertise", "topic": self.topic_, "type": msg_type})
        self.ws_.send(pub_msg)

    def unadvertise_topic(self):
        """ Unadvertise the topic to be published on the server """
        pub_msg = dumps({"op": "unadvertise", "topic": self.topic_})
        self.ws_.send(pub_msg)

    def connect(self, server):
        """ Establish a connection to the server """
        try:
            print "Connecting to ", server, "...",
            self.ws_ = create_connection(server)
            print "connected."
            return True
        except:
            print "cannot connect to the server."
            return False

    def disconnect(self):
        """ Close the connection """
        if self.ws_:
            self.unadvertise_topic()
            self.ws_.close()

    def encode(self, string):
        """ PNG-compress the string into a square RGB image padded with '\n', return the b64 encoded bytes """
        length = len(string)
        width = floor(sqrt(length/3.0))
        height = ceil((length/3.0) / width)
        bytes_needed = int(width * height * 3)
        while length < bytes_needed:
            string += '\n'
            length += 1
        i = Image.fromstring('RGB', (int(width), int(height)), string)
        buff = StringIO()
        i.save(buff, "png")
        return standard_b64encode(buff.getvalue())

def usage():
    return "%s server_ip_addr:port" % sys.argv[0]
    
if __name__ == "__main__":
    if len(sys.argv) == 1:
        print usage()
        sys.exit(1)
    else:
        ip_addr = sys.argv[1]
    server = 'ws://' + ip_addr
    try:
        rospy.init_node('client_pointcloud_xmit')
        client_pointcloud_xmit = ClientPointCloudXmit()
        if client_pointcloud_xmit.connect(server):
            client_pointcloud_xmit.advertise_topic()
            rospy.on_shutdown(client_pointcloud_xmit.disconnect)
            rospy.Subscriber(client_pointcloud_xmit.input_, PointCloud, \
                             client_pointcloud_xmit.cloud_callback, queue_size=1)
            rospy.spin()
    except rospy.ROSInterruptException:
        pass

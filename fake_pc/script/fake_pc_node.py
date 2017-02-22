#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import PointCloud as pc
from sensor_msgs.msg import PointField as pf
from geometry_msgs.msg import Point32 as p32
from sensor_msgs.msg import *
from sensor_msgs import *

class fake_pc():
    def __init__(self):
        rospy.init_node('fake_pc')
        self.pub = rospy.Publisher('/robot_model_to_pointcloud/robot_cloud2',pc2,queue_size=10)
        self.rate = 1
        self.run()

    def run(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            r.sleep()

            c = pc()
            # c.points.reserve(5000)

            msg = pc2()
            _p32 = p32()

            msg.header.stamp = rospy.get_rostime()
            # msg.header.frame_id = 'kinect_link'
            msg.header.frame_id = 'base_link'
            msg.height = 1
            msg.width = 0

            # msg.fields = []

            msg.fields.append(pf())
            msg.fields[0].name = 'x'
            msg.fields[0].offset = 0
            msg.fields[0].datatype = 7
            msg.fields[0].count = 1

            msg.fields.append(pf())
            msg.fields[1].name = 'y'
            msg.fields[1].offset = 4
            msg.fields[1].datatype = 7
            msg.fields[1].count = 1

            msg.fields.append(pf())
            msg.fields[2].name = 'z'
            msg.fields[2].offset = 8
            msg.fields[2].datatype = 7
            msg.fields[2].count = 1

            msg.fields.append(pf())
            msg.fields[3].name = 'intensity'
            msg.fields[3].offset = 8
            msg.fields[3].datatype = 7
            msg.fields[3].count = 1

            # link = ChannelFloat32
            # link.name = 'intensity'
            # link.values.reserve(5000)

            # c.channels.append(link)
            # msg.fields.append(link)
            # convertPointCloudToPointCloud2(c,msg)

            self.pub.publish(msg)

if __name__ == '__main__':
    fp = fake_pc()

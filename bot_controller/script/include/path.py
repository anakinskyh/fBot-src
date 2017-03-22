#!/usr/bin/env python

from geometry_msgs.msg import Pose

default_path = []

pose = Pose()
(pose.position.x,pose.position.y,pose.position.z) = (0,0,0)
(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w) = (0,0,0,1)
default_path.append(pose)

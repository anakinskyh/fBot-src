#!/usr/bin/env python

import rospy
from include import goal_setter,path
from geometry_msgs.msg import Pose

class n_point_move():
    def __init__(self):
        self.gs = goal_setter()
        self.path = default_path

    def clear_path(self):
        self.path = []

    def append_path(self,px = 0,py = 0,pz = 0,ox=0,oy=0,oz =0,ow = 1):
        pose = Pose()
        (pose.position.x,pose.position.y,pose.position.z) = (px,py,pz)
        (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w) = (ox,oy,oz,ow)

        self.path.append(pose)

    def follow_path(self,n_times = 1):
        for i in range(0,n_times):
            self.gs.follow_path(self.path)

if __name__ == '__main__':
    rospy.init_node('n_point_move')
    rospy.loginfo('n_point_move start')

    npmove = n_point_move()
    npmove.follow_path()

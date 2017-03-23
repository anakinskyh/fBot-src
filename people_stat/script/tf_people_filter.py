#!/usr/bin/env python

import rospy
import tf

class tf_people_filter():
    def __init__(self):

        rospy.init_node('tf_people_filter')

        self.cam_frame = rospy.get_param('cam_frame','openni_depth_frame')

    def 

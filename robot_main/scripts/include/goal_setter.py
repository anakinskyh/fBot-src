#!/usr/bin/env python
# For move robot around the map

import rospy

class goal_setter():

    def __init__(self,move_base = 'move_base'):
        rospy.loginfo('goal setter init')
        self.move_base = move_base # for luanch action

    def move_to(self,path):
        # move to goal
        # return status [bool]
        # status : to be implemented
        rospy.loginfo('move to somewhere')

    def follow_path(self,paths):
        # pass this function set of goal
        # return set of status [bool]
        # dependensy on : move_to
        # status : to be implemented
        rospy.loginfo('follow the paths')

    def load_path(self,file_path):
        # passing file that collect paths to follow
        # return status [bool]
        # status : to be implemented
        rospy.loginfo('load file')

    
#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal

class goal_setter():
    def __init__(self,debug_mode = False,move_base='move_base',frame = 'base_link'):
        self.move_base = move_base
        self.frame = frame
        self.debug_mode = debug_mode

        self.client = actionlib.SimpleActionClient(self.move_base,MoveBaseAction)
        self.client.wait_for_server()

    """ Pass goal to me and  """
    def move_to(self,pose):
        if self.debug_mode:
            rospy.loginfo('go to somewhere ')

        mb_goal = MoveBaseGoal()

        # set header
        mb_goal.target_pose.header.frame_id = self.frame
        mb_goal.target_pose.header.stamp = rospy.get_rostime()

        # set pose
        mb_goal.target_pose.pose = pose

        rospy.loginfo('go to ...')

        self.client.send_goal(mb_goal)

        self.client.wait_for_result()

        if self.debug_mode:
            rospy.loginfo('STATUS : '+self.client.get_result())

        return self.client.get_result()

    # path is a set of pair position and orientation
    def follow_path(self,path):
        # for i_goal in path:

        rospy.loginfo('follow path done')

if __name__ == '__main__':
    rospy.init_node('simple_goal_setter')
    gs = goal_setter()

    # gs.move_to()

#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from geometry_msgs.msg import Pose

class goal_setter():
    def __init__(self,debug_mode = True,move_base='move_base',frame = 'map'):
        self.move_base = move_base
        self.frame = frame
        self.debug_mode = debug_mode

        self.client = actionlib.SimpleActionClient(self.move_base,MoveBaseAction)
        self.client.wait_for_server()

    """ Pass goal to me and  """
    def move_to(self,pose,wait = False):
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

        if wait:
            rospy.loginfo('logx')
            self.client.wait_for_result()

            self.status = self.client.get_state()

            if self.debug_mode:
                rospy.loginfo('STATUS : '+str( actionlib.SimpleGoalState.ACTIVE ))

            return self.status

        rospy.loginfo('log')

    # path is a set of pair position and orientation
    def follow_path(self,path):
        # for i_goal in path:

        rospy.loginfo('follow path done')
        for pose in path:
            status = self.move_to(pose)
            while not rospy.is_shutdown() and self.client.get_state()!=3:
                rospy.loginfo(self.client.get_state())
                rospy.sleep(0.2)

if __name__ == '__main__':
    rospy.init_node('simple_goal_setter')
    gs = goal_setter()
    path = []
    p = Pose()
    (p.position.x,p.position.y,p.position.z) = (5.43,4.22,0.00)
    (p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w) = (0,0,0.98,-0.187)
    path.append(p)

    p = Pose()
    (p.position.x,p.position.y,p.position.z) = (7.9,5.8,0.00)
    (p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w) = (0,0,0.65,0.752)
    path.append(p)

    p = Pose()
    (p.position.x,p.position.y,p.position.z) = (1.1652,9.063626,0.00)
    (p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w) = (0,0,0.018931,0.99982)
    path.append(p)

    p = Pose()
    (p.position.x,p.position.y,p.position.z) = (1.2218,2.3134,0.0)
    (p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w) = (0,0,0.65,0.7596)
    path.append(p)

    rospy.loginfo(path)

    gs.follow_path(path)

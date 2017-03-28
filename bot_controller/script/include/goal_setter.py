#!/usr/bin/env python

import rospy
import actionlib
import thread

from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal,MoveBaseActionFeedback
from geometry_msgs.msg import Pose
from people_msgs.msg import People
from actionlib_msgs.msg import GoalStatus,GoalStatusArray
import numpy as np

class goal_setter():
    def __init__(self,debug_mode = True,move_base='move_base',frame = 'map',use_human_monitor = True,human_dist = 10.0):
        self.move_base = move_base
        self.frame = frame
        self.debug_mode = debug_mode
        self.use_human_monitor = use_human_monitor
        self.human_dist = human_dist

        self.client = actionlib.SimpleActionClient(self.move_base,MoveBaseAction)
        self.client.wait_for_server()


        rospy.Subscriber('/move_base/status',GoalStatusArray,self.callback_status)

        # subscribe people and pose
        if self.use_human_monitor:
            rospy.Subscriber('/people',People,self.callback_people)
            rospy.Subscriber('/move_base/feedback',MoveBaseActionFeedback,self.callback_feedback)
            rospy.Subscriber('/move_base/status',GoalStatusArray,self.callback_status)

    def callback_pose(self,msg):
        self.current_pose = msg

    def callback_people(self,msg):
        self.people = msg

    def callback_feedback(self,msg):
        self.mb_base_pos = msg.feedback.base_position.pose.position
        self.mb_status = msg.status.status

    def callback_status(self,msg):
        self.cb_status = msg
        if len(msg.status_list) > 0:
            self.cb_status = msg.status_list[len(msg.status_list)-1].status

    """ Pass goal to me and  """
    def move_to(self,pose,wait = False,new_goal = True):
        if new_goal:
            self.current_goal = pose
            # thread.start_new_thread(self.human_monitor,())

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
            # rospy.loginfo('logx')
            self.client.wait_for_result()

            self.status = self.client.get_state()

            if self.debug_mode:
                    rospy.loginfo('STATUS : '+str( actionlib.SimpleGoalState.ACTIVE ))

            return self.status

    # path is a set of pair position and orientation
    def follow_path(self,path):
        # for i_goal in path:

        # rospy.loginfo('follow path done')
        for pose in path:
            status = self.move_to(pose)
            while not rospy.is_shutdown() and self.client.get_state()!=3:
                rospy.loginfo(self.client.get_state())
                rospy.sleep(0.2)

    def get_status(self):

        try:
            return self.mb_status
        except:
            rospy.logerr('get_status error')
        return -1

    def get_current_goal(self):

        try:
            return self.current_goal
        except:
            rospy.logerr('get_current_goal error')
        return -1

    def human_monitor(self):
        rospy.loginfo('human monitor')
        stop = False

        while (self.mb_status != GoalStatus.SUCCEEDED \
            and self.mb_status != GoalStatus.ABORTED \
            and self.mb_status != GoalStatus.LOST \
            and self.mb_status != GoalStatus.REJECTED \
            ) or stop \
            :
            nearest_dist = 100000.00
            np_mb_base_pos = np.array([self.mb_base_pos.x,self.mb_base_pos.y,self.mb_base_pos.z])
            for person in self.people.people:
                person_pos = person.position
                np_person_pos = np.array([person_pos.x,person_pos.y,person_pos.z])

                dist = np.linalg.norm(np_mb_base_pos-np_person_pos)

                nearest_dist = min(nearest_dist,dist)

            if nearest_dist <= self.human_dist:
                stop = True

                self.move_to(self.mb_base_pos,new_goal = True)
            else:
                stop = False



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

#! /usr/bin/env python
import rospy
from geometry.msgs import PoseStamped
import actionlib
from actionlib.msgs import GoalStatusArray
from move_base_msgs.msgs import MoveBaseAction,MoveBaseGoal

class path_setter():

    def __init__(self,pub_topic = '/goal', sub_topic = ''):
        self.pub_topic = pub_topic
        self.pub = rospy.Publisher('goal',PoseStamped,queue_size=10)

    def init_param(self):
        self.pub = rospy.get_param('~pub','')

    def move_to(self,goal): # goal = [x y z w]

        self.ac = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.ac.wait_for_server()

        self.goal = MoveBaseGoal()

        self.goal.target_pose.header.frame_id = 'base_link'
        self.goal.target_pose.header.stamp = rospy.Time.now()

        self.goal.target_pose.pose.position.x = goal.x
        self.goal.target_pose.pose.position.y = goal.y
        self.goal.target_pose.pose.orientation.w = goal.w
        self.goal.target_pose.pose.orientation.z = goal.z

        self.ac.sendGoal(self.goal)
        self.ac.waitForResult()

        rospy.loginfo('%s',self.ac.result())

    def get_status(self):
        client = actionlib.SimpleActionClient('')

if __name__ == '__main__':
    ps = path_setter()
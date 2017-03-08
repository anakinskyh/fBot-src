#! /usr/bin/env python
import rospy
from geometry_msgs.msgs import PoseStamped
import actionlib
from actionlib.msgs import GoalStatusArray
from move_base_msgs.msgs import MoveBaseAction,MoveBaseGoal

#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseActionFeedback
from nav_msgs.msg import Path,Odometry
from people_msgs.msg import People
from std_msgs.msg import Float64 as Float,Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import tf
import thread

class human_monitor():

    def __init__(self):

        rospy.init_node('human_monitor')

        # get param
        self.human_lin_scale = rospy.get_param('~human_lin_scale',1.0)
        self.human_ang_scale = rospy.get_param('~human_ang_scale',1.0)
        self.human_dist = rospy.get_param('~human_dist',0.0)

        self.mb_start = False

        # Subscriber
        rospy.Subscriber('/move_base/feedback',MoveBaseActionFeedback,self.callback_feedback)
        # rospy.Subscriber('/odom',PoseWithCovarianceStamped,self.callback_odom)
        rospy.Subscriber('/people',People,self.callback_people)

        # Publisher
        self.lin_scale_pub = rospy.Publisher('lin_scale',Float,queue_size=10)
        self.ang_scale_pub = rospy.Publisher('ang_scale',Float,queue_size=10)
        self.cancel_pub = rospy.Publisher('cancel_scale',Bool,queue_size=10)

        # Message
        self.lin_msg = Float()
        self.lin_msg.data = self.human_lin_scale

        self.ang_msg = Float()
        self.ang_msg.data = self.human_ang_scale

        self.cancel_msg = Bool()
        self.cancel_msg.data = True

        thread.start_new_thread(self.update_tf,())

    def callback_feedback(self,msg):
        self.mb_start = True
        self.mb_base_pos = msg.feedback.base_position.pose
        self.mb_status = msg.status.status

    def callback_people(self,msg):
        self.people_start = True
        self.people = msg
        # rospy.loginfo(msg)

    def callback_odom(self,msg):
        self.mb_start = True
        self.mb_base_pos = msg.pose.pose
        # self.mb_status = msg.status.status

    def update_tf(self):
        listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))
                # rospy.loginfo("{} {}".format(trans,rot) )

                msg = Odometry()
                msg.header.frame_id = 'map'

                msg.pose.pose.position.x = trans[0]
                msg.pose.pose.position.y = trans[1]
                msg.pose.pose.position.z = trans[2]

                msg.pose.pose.orientation.x = rot[0]
                msg.pose.pose.orientation.y = rot[1]
                msg.pose.pose.orientation.z = rot[2]
                msg.pose.pose.orientation.w = rot[3]

                # self.demo_pub.publish(msg)
                self.mb_base_pos = msg.pose.pose
                # rospy.loginfo(self.mb_base_pos)
            except:
                rospy.loginfo('err')

    def monitor(self):

        slow = False
        # rospy.loginfo('run laew na ja')

        while not rospy.is_shutdown():

            if not self.mb_start or not self.people_start:
                continue
            # rospy.loginfo('run laew na ja')
            nearest_dist = 100000.00
            np_mb_base_pos = np.array([self.mb_base_pos.position.x,self.mb_base_pos.position.y,self.mb_base_pos.position.z])
            for person in self.people.people:
                person_pos = person.position
                np_person_pos = np.array([person_pos.x,person_pos.y,person_pos.z])

                dist = np.linalg.norm(np_mb_base_pos-np_person_pos)

                nearest_dist = min(nearest_dist,dist)

            # if nearest_dist < 100000.00:
            #     rospy.loginfo('detect dist : {}'.format(nearest_dist) )

            if nearest_dist > self.human_dist and slow:
                slow = False
                self.cancel_pub.publish(self.cancel_msg)

            elif nearest_dist <= self.human_dist:
                # rospy.loginfo('slow')
                slow = True
                self.lin_scale_pub.publish(self.lin_msg)
                self.ang_scale_pub.publish(self.ang_msg)

if __name__ == '__main__':
    mnt = human_monitor()
    mnt.monitor()

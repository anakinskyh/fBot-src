#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64 as Float,Bool

class vel_scaler():

    def __init__(self):

        rospy.init_node('vel_scaler')

        # read param
        self.sub_cmd_vel = rospy.get_param('~sub_cmd_vel','smooth_cmd_vel')
        self.pub_cmd_vel = rospy.get_param('~pub_cmd_vel','cmd_vel')

        self.lin_scale_topic = rospy.get_param('~lin_scale_topic','lin_scale')
        self.ang_scale_topic = rospy.get_param('~ang_scale_topic','ang_scale')

        self.default_lin_scale = rospy.get_param('~default_lin_scale',1)
        self.default_ang_scale = rospy.get_param('~default_ang_scale',1)

        self.lin_scale = self.default_lin_scale
        self.ang_scale = self.default_ang_scale

        # publisher
        self.cmd_pub = rospy.Publisher(self.pub_cmd_vel,Twist,queue_size = 10)

        # Subscriber
        rospy.Subscriber(self.sub_cmd_vel,Twist,self.callback_sub_vel)
        rospy.Subscriber(self.lin_scale_topic,Float,self.callback_lin_scale)
        rospy.Subscriber(self.ang_scale_topic,Float,self.callback_ang_scale)
        rospy.Subscriber('cancel_scale',Bool,self.callback_cancel_scale)

        rospy.spin()

    def callback_sub_vel(self,msg):

        msg.linear.x = msg.linear.x*self.lin_scale
        msg.linear.y = msg.linear.y*self.lin_scale
        msg.linear.z = msg.linear.z*self.lin_scale

        msg.angular.x = msg.angular.x*self.ang_scale
        msg.angular.y = msg.angular.y*self.ang_scale
        msg.angular.z = msg.angular.z*self.ang_scale

        self.cmd_pub.publish(msg)

    def callback_lin_scale(self,msg):
        self.lin_scale = msg.data

    def callback_ang_scale(self,msg):
        self.ang_scale = msg.data

    def callback_cancel_scale(self,msg):
        self.lin_scale = self.default_lin_scale
        self.ang_scale = self.default_ang_scale

if __name__ == '__main__':
    vs = vel_scaler()

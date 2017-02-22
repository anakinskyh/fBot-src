#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import thread
from include import turn_signal

class light():
    def callback(self,msg):
        if msg.linear.x >0:
            self.vel_x = (self.vel_x + msg.linear.x)/2
        else:
            self.vel_x = 0

        if msg.angular.z == 0:
            self.vel_z = 0
        elif msg.angular.z > 0 and self.vel_z <= 0:
            self.vel_z = msg.angular.z
        elif msg.angular.z < 0 and self.vel_z >= 0:
            self.vel_z = msg.angular.z
        else:
            self.vel_z = (self.vel_z + msg.angular.z)/2

    def get_status(self):
        # analyse twist
        if self.vel_x == 0 and self.vel_z == 0:
            self.ts_state = 'stop'
            # rospy.loginfo('what')
        elif self.vel_x == 0: # turn left or right
            if self.vel_z > 0: # left
                self.ts_state = 'left'
            else:
                 self.ts_state = 'right'
        elif self.vel_z == 0: # go up
            self.ts_state = 'top'
        else :
            if self.vel_z > 0:
                self.ts_state = 'top left'
            else :
                self.ts_state = 'top right'

    def update_status(self):

        update = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            update.sleep()

            self.get_status()
            # rospy.loginfo('status')
            self.driver.change_state(self.ts_state,self.vel_x,self.vel_z)

    def update_light(self):
        
        self.driver.set_rate(self.light_rate)

        self.driver.update_light()


    def set_pub(self):
        self.pub = rospy.Publisher(self.pub_topic, TsState)


    def listener(self):
        
        rospy.loginfo('start')

        # subscribe
        rospy.loginfo('subscribe: %s',self.sub_topic)
        rospy.Subscriber(self.sub_topic, Twist, self.callback)

    def get_param(self):
        self.sub_topic = rospy.get_param('~sub_topic','/cmd_vel')

        self.dev = rospy.get_param('~dev','/dev/arduino')
        self.baudrate = rospy.get_param('~baudrate',9600)

        self.update_rate = rospy.get_param('~update_rate',10.0)
        self.light_rate = rospy.get_param('~light_rate',2.0)



    def init(self):

        # start
        rospy.init_node('light')

        # init
        self.ts_state = 'stop'

        self.vel_x = 0.0
        self.vel_z = 0.0

        self.prev_cmd = 'null'

        rospy.loginfo('start node')

        self.get_param()

        self.listener()

        #set pixel_driver
        self.driver = turn_signal.turn_signal(self.dev,self.baudrate)

        # self.write_ser()
        thread.start_new_thread(self.update_light,())
        thread.start_new_thread(self.update_status,())

        rospy.spin()

if __name__ == '__main__':
    l = light()
    l.init()

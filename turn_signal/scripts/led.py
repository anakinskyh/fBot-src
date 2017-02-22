#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import serial
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class light():
    def callback(self,msg):

        # debug
        rospy.loginfo('state: %s',self.state)
        # print 'state: %s'%(self.state)

        self.linear = msg.linear
        self.angular = msg.angular

        self.linear_is_zero = self.linear.x == 0
        self.angular_is_zero = self.angular.z == 0

        if self.state == 'set stop':
            return

        #cmd = "[0,40,0,0,0,1]" #default is set zero
        stop = False

        if self.linear_is_zero and self.angular_is_zero:
            if self.state == 'stop':
                stop = True # set break
            elif self.zero_cont >= self.max_zero_cont:
                self.state = 'set stop'
                stop = True # set break
            else:
                self.state = 'count'
                self.zero_cont+=1
                return
        else:
            self.zero_cont = 0
            self.state = 'run'

            if self.linear_is_zero:
                if self.angular.z > 0:
                    cmd = "[0,19,"
                else:
                    cmd = "[20,39,"

            elif self.angular_is_zero:
                cmd = '[15,24,'

            else :
                if self.angular.z > 0:
                    cmd = "[5,24,"
                else:
                    cmd = "[15,34,"

        # send signal
        if self.state == 'set stop':
            rospy.loginfo('[0,40,%s\n'%(self.break_color))
            self.ser.write('[0,40,%s\n'%(self.break_color))
            # rospy.sleep(1)
            self.ser.write(self.blank)
            
            self.state = 'stop'
            return

        if stop == True:
            self.ser.write(self.blank)
            
            self.state = 'stop'
            return

        if self.state == 'run':
            self.ser.write(self.blank)
            self.ser.write('%s%s\n'%(cmd,self.move_color))

    def listener(self):
        
        rospy.loginfo('start')
        # print('start')


        # subscribe
        rospy.loginfo('subscribe: %s',self.sub_topic)
        rospy.Subscriber(self.sub_topic, Twist, self.callback)

        # open serial port
        try:
            self.ser = serial.Serial(self.dev,baudrate=9600,timeout=10)
        except :
            rospy.loginfo('unable to connect to port %s',self.dev)
            pass

        rospy.spin()

    def get_param(self):
        self.sub_topic = rospy.get_param('~sub_topic','/cmd_vel')
        self.use_light_ratio = rospy.get_param('~use_light_ratio',False)
        self.light_ratio = rospy.get_param('~light_ratio',0.0)
        self.rate = rospy.get_param('~light_rate',10.0)
        self.dev = rospy.get_param('~dev','/dev/ttyACM0')
        self.max_zero_cont = rospy.get_param('~max_zero_cont',5)

        self.blank = rospy.get_param('~blank','[0,40,0,0,0,1]\n')
        self.move_color = rospy.get_param('~move_color','0,20,0,1]')
        self.break_color = rospy.get_param('~break_color','20,0,0,1]')


    def init(self):

        # start
        rospy.init_node('light')

        rospy.loginfo('start node')

        self.get_param()

        # init for listener
        self.zero_cont = 0
        self.state = 'stop'

        self.listener()

if __name__ == '__main__':
    l = light()
    l.init()

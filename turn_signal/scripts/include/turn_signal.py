#!/usr/bin/env python

import sys
import rospy
import serial
import time
import numpy as np
# from turn_signal.msg import TsState
import pixel_driver
import signal_template as st

class turn_signal():
    def __init__(self,dev = '/dev/arduino',boudRate = 115200,size = 40):
        self.dev = dev
        self.boudRate = boudRate
        self.size = 40

        self.prevdir = 'center'

        self.ts_signal = 'stop'
        self.vel_x = 0.0
        self.vel_z = 0.0

        self.driver = pixel_driver.pixel_driver(dev=dev,boudRate = boudRate,size =size)

    def change_state(self,ts_signal,vel_x,vel_z):
        self.ts_signal = ts_signal
        self.vel_x = vel_x
        self.vel_z = vel_z

        # rospy.loginfo('change : %s %f %f',self.ts_signal,self.vel_x,self.vel_z)

    def set_rate(self,rate):
        self.rate = rospy.Rate(rate)
    
    def update_light(self):
        while not rospy.is_shutdown():
            # rospy.loginfo('%s',self.ts_signal)
            if self.ts_signal == 'stop':
                self.stop()

            elif self.ts_signal == 'left':
                self.left()

            elif self.ts_signal == 'right':
                self.right()

            elif self.ts_signal == 'top':
                self.top()

            elif self.ts_signal == 'top left':
                self.top_left()

            elif self.ts_signal == 'top right':
                self.top_right()
            
            else:
                rospy.loginfo('unmatch : %s',self.ts_signal)
                self.top()

    def stop(self):
        self.driver.set_range(left=0,right=40,color=[20,0,0,1])
        self.driver.show()

        rospy.sleep(0.2)

        self.driver.set_range(left=0,right=40,color=[0,0,0,1])
        self.driver.show()

        # rospy.loginfo('enter stop')

        while self.ts_signal == 'stop' and not rospy.is_shutdown():
            self.rate.sleep()
            # rospy.loginfo('a')

    def left(self):
        
        signal = st.left

        while self.ts_signal == 'left' and not rospy.is_shutdown():

            self.driver.set_by_colorlist(signal)
            self.driver.show()

            signal = np.roll(signal,-1,axis=0)

            self.rate.sleep()

    def right(self):
        
        signal = st.right

        while self.ts_signal == 'right' and not rospy.is_shutdown():

            self.driver.set_by_colorlist(signal)
            self.driver.show()


            self.rate.sleep()

            signal = np.roll(signal,1,axis=0)

    def top(self):
        signal = st.top0
        # rospy.loginfo('top')

        while self.ts_signal == 'top' and not rospy.is_shutdown():

            self.driver.set_by_colorlist(signal)
            self.driver.show()
            self.rate.sleep()
            
    def top_left(self):

        signal = st.left

        while self.ts_signal == 'left' and not rospy.is_shutdown():

            self.driver.set_by_colorlist(signal)
            self.driver.show()

            signal = np.roll(signal,-1,axis=0)

            self.rate.sleep()

    def top_right(self):

        signal = st.right

        while self.ts_signal == 'top right' and not rospy.is_shutdown():

            self.driver.set_by_colorlist(signal)
            self.driver.show()


            self.rate.sleep()

            signal = np.roll(signal,1,axis=0)



if __name__ == '__main__':
    ts = turn_signal()
    ts.turn_right()

        
        

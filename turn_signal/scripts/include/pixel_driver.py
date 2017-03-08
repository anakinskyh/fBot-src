#!/usr/bin/env python

import sys
import rospy
import serial
import time
import numpy as np

class pixel_driver():
    def __init__(self,dev = '/dev/ttyACM0',boudRate = 9600,size = 40):
        self.dev = dev
        self.boudRate = boudRate
        self.size = size
        try:
            self.ser = serial.Serial(self.dev,self.boudRate,timeout=10)
            time.sleep(3)
        except :
            print 'unable to connect to port %s',self.dev
            rospy.loginfo('unable to connect to port %s',self.dev)
            pass

    def write_serial(self,msg):
        msg_out = msg+'\n'
        # print 'write %s'%(msg_out)
        try:
            # rospy.loginfo('%s %i',self.dev,self.boudRate)
            self.ser = serial.Serial(self.dev,self.boudRate,timeout=10)
            # self.ser.open()
            self.ser.write(msg_out)
            time.sleep(0.05)
        except :
            print 'unable to connect to port %s',self.dev
            rospy.loginfo('unable to connect to port %s',self.dev)
            pass

    def show(self):
        self.write_serial('0')

    def reset_range(self,left = 0,right = 40):
        cmd = '1 %i %i 0 0 0 1'%(left,right)
        self.write_serial(cmd)
        self.show()

    def reset_all(self):
        self.reset_range(0,self.size-1)

    def set_range(self,left = 0,right =40,color=[40,40,40,1]):
        cmd = ' '.join(str(x) for x in color)
        cmd = '1 %i %i %s'%(left,right,cmd)
        self.write_serial(cmd)

    def set(self,pos=0,color=[40,40,40,1]):
        self.set_range(left=pos,right=pos,color = color)

    def set_all(self,color=[40,40,40,1]):
        self.set_range(left=0,right=self.size-1,color=color)

    def set_by_list(self,ledlist=[0],color=[40,40,40,1]):
        for x in ledlist:
            self.set(pos=x,color=color)

    def set_by_colorlist(self,colorlist,start=0):
        cmd = '2 %i %i'%(start,colorlist.shape[0])
        if colorlist.shape[1]<4:
            return

        for x in colorlist:
            tmp = ' '.join(str(y) for y in x)
            cmd = '%s %s'%(cmd,tmp)

        # print cmd
        self.write_serial(cmd)
            
        # time.sleep(0.1)

def f(x,y):
    return 0

if __name__ == '__main__':
    pd = pixel_driver(dev = '/dev/arduino',boudRate = 115200)
    pd.reset_all()
    pd.set_by_list(ledlist=[0,2,5],color=[40,0,40,1])
    pd.show()
    # time.sleep(0.5)   
    a = np.array([(50,10,0,1),(0,10,50,1)])
    pd.set_by_colorlist(a,start = 10)
    pd.show()
    # time.sleep(1)
    b = np.fromfunction(f,(40,4),dtype=int)
    
    # print b
    for i in range(0,100):
        time.sleep(0.01)
        pd.set_by_colorlist(b)
        
        # time.sleep(2)
        pd.show()

        b = np.roll(b,-1,axis=0)



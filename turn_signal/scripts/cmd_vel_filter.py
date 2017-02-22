#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class cmd_filterer():
    def make_smoother(self,twist):
        if twist.linear.x>self.vel_x:
            self.vel_x = min(twist.linear.x,self.vel_x+self.max_l_acc)
        else:
            self.vel_x = max(twist.linear.x,self.vel_x-self.max_l_acc)

        if twist.angular.z>self.vel_z:
            self.vel_z = min(twist.angular.z,self.vel_z+self.max_a_acc)
        else:
            self.vel_z = max(twist.angular.z,self.vel_z-self.max_a_acc)

    def callback(self,twist):
        self.turn_pub.publish(twist)

        rospy.sleep(self.delay)

        # dont focus now this is will always be pass
        if self.smoother and False:
            self.make_smoother(twist)
            twist.linear.x = self.vel_x
            twist.angular.z = self.vel_z

        

        #tell to some class

        self.pub.publish(twist)

    def setparam(self):
        self.delay = rospy.get_param('~delay',0.0)
        self.queue_size = rospy.get_param('~queue_size',10)
        self.sub_topic = rospy.get_param('~sub_topic','raw_cmd_vel')
        self.pub_topic = rospy.get_param('~pub_topic','cmd_vel')

        self.smoother = rospy.get_param('~smoother',False)
        self.max_l_acc = rospy.get_param('~max_l_acc',0.1)
        self.max_a_acc = rospy.get_param('~max_a_acc',0.1)

    def init(self):

        # init node
        rospy.init_node('cmd_filter', anonymous=True)
        self.setparam()

        if self.smoother:
            self.vel_x = 0.0
            self.vel_z = 0.0

        #publish default 'cmd_vel'
        self.pub = rospy.Publisher(self.pub_topic, Twist, queue_size = self.queue_size)
        self.turn_pub = rospy.Publisher('turn_cmd_vel', Twist, queue_size = self.queue_size)

        #subscriber default 'raw_cmd_vel'
        rospy.Subscriber(self.sub_topic, Twist, self.callback,queue_size=1, buff_size=2**24)

        rospy.spin( )
        # rospy.asyncSpin()

if __name__ == '__main__':
    cf = cmd_filterer()
    while not rospy.is_shutdown():
        cf.init()

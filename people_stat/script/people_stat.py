#!/usr/bin/env python
import rospy
from people_msgs.msg import People,Person
from nav_msgs.msg import Odometry as odom
import tf
import numpy as np

class people_stat():
    def __init__(self):
        self.get_param()
        rospy.init_node('people_stat')

        # set up
        self.nzero = 1e-4
        self.listener = tf.TransformListener()
        self.publisher()

        self.wait_for_tf()


    def get_param(self):
        rospy.loginfo('get_param')
        self.max_people = rospy.get_param('max_people',15)
        self.frame_id = rospy.get_param('camera_frame_id','openni_depth_frame')
        self.people_topic = rospy.get_param('people_topic','people')
        # self.sensitive_dist = rospy.get_param('sensitive_dist',2.0)
        # self.minimum_vel = rospy.get_param('minimum_vel',2.0)
        self.change = rospy.get_param('change',0.05)

    def publisher(self):
        self.pub = rospy.Publisher(self.people_topic,People,queue_size=10)

    def wait_for_tf(self):
        rospy.loginfo('start')
        self.rate = rospy.Rate(10)
        self.people_prev = {}


        while not rospy.is_shutdown():

            people_cnt = 0

            # create msg
            people_msg = People()
            people_msg.header.stamp = rospy.Time()
            people_msg.header.frame_id = self.frame_id

            self.people_now = {}

            for i in range(1,1+self.max_people):
                try:
                    child = 'neck_'+str(i)
                    time = rospy.Time(0)
                    dur = rospy.Duration.from_sec(0.8)
                    
                    (pose,qt) = self.listener.lookupTransform(self.frame_id,child,time)
                    (lin,ang) = self.listener.lookupTwist(self.frame_id,child,time,dur)
                    
                    if all(v == 0 for v in lin) or all(v == 0 for v in ang):
                        continue

                    # linear vel is too small
                    if np.linalg.norm(np.array(lin)-0) < self.nzero:
                        continue

                    # rospy.loginfo(child)
                    # rospy.loginfo(time)
                    # rospy.loginfo(lin)
                    # rospy.loginfo(ang)

                    person_msg = Person()
                    person_msg.name  = 'vel_%s' % (child)
                    (person_msg.position.x,person_msg.position.y,person_msg.position.z) = pose


                    # velocity is too sensitive
                    # if np.linalg.norm(np.array(lin)-0) >= self.minimum_vel:
                        # (person_msg.velocity.x,person_msg.velocity.y,person_msg.velocity.z) = lin

                    if child in self.people_prev:
                        (person_msg.velocity.x,person_msg.velocity.y,person_msg.velocity.z) = \
                            (self.people_prev[child].velocity.x*(1-self.change)+self.change*lin[0], \
                            self.people_prev[child].velocity.y*(1-self.change)+self.change*lin[1], \
                            self.people_prev[child].velocity.z*(1-self.change)+self.change*lin[2])
                    else:
                        (person_msg.velocity.x,person_msg.velocity.y,person_msg.velocity.z) = \
                            (self.change*lin[0], \
                            self.change*lin[1], \
                            self.change*lin[2])

                    self.people_now[child] = person_msg
                    people_msg.people.append(person_msg)

                    people_cnt += 1


                except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
                    continue

            self.people_prev = self.people_now

            if people_cnt <= 0:
                continue
            
            rospy.loginfo('publish di wa')
            rospy.loginfo(people_msg)
            self.prev_people = people_msg

            self.pub.publish(people_msg)

            self.rate.sleep()
            

    def send_vel(self,msg):
        rospy.loginfo('calculate_vel')

if __name__ == '__main__':
    ps = people_stat()
#!/usr/bin/env python
import rospy
from people_msgs.msg import People,Person
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
        self.sensitive_dist = rospy.get_param('sensitive_dist',2.0)

    def publisher(self):
        self.pub = rospy.Publisher(self.people_topic,People,queue_size=10)

    def wait_for_tf(self):
        rospy.loginfo('start')
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            people_cnt = 0

            # create msg
            people_msg = People()
            people_msg.header.stamp = rospy.Time()
            people_msg.header.frame_id = self.frame_id

            for i in range(1,1+self.max_people):
                try:
                    child = 'neck_'+str(i)
                    time = rospy.Time(0)
                    dur = rospy.Duration.from_sec(0.01)
                    
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
                    (person_msg.velocity.x,person_msg.velocity.y,person_msg.velocity.z) = lin

                    people_msg.people.append(person_msg)

                    people_cnt += 1


                except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
                    continue

            if people_cnt <= 0:
                continue
            
            rospy.loginfo('publish di wa')
            rospy.loginfo(people_msg)

            self.pub.publish(people_msg)

            self.rate.sleep()
            

    def send_vel(self,msg):
        rospy.loginfo('calculate_vel')

if __name__ == '__main__':
    ps = people_stat()
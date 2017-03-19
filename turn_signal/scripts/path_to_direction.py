#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
import tf
import numpy as np
from math import pi
import thread
from include import turn_signal

class path_to_direction():
    def __init__(self):
        rospy.init_node('path_to_direction')

        # read param
        self.update_frequency = rospy.get_param('update_frequency',4)
        self.ratio = rospy.get_param('ratio',0.6)
        self.lookup = rospy.get_param('lookup',80)
        self.turn_threshold = rospy.get_param('turn_threshold',0.8)
        self.listen_topic = rospy.get_param('listen_topic','/move_base/DWAPlannerROS/global_plan')
        self.lookup_plan = rospy.get_param('lookup_plan','lookup_plan')

        # self.sub_topic = rospy.get_param('~sub_topic','/cmd_vel')

        self.dev = rospy.get_param('~dev','/dev/arduino')
        self.baudrate = rospy.get_param('~baudrate',115200)

        self.update_rate = rospy.get_param('~update_rate',5.0)
        self.light_rate = rospy.get_param('~light_rate',10.0)

        # init
        self.is_update = False
        #set pixel_driver
        self.driver = turn_signal.turn_signal(self.dev,self.baudrate)
        self.ts_state = 'stop'
        self.angle = 0.00
        self.stop_counter = 0

        # listen
        rospy.Subscriber(self.listen_topic,Path,self.callback)

        # pubblish
        self.lookup_plan_pub = rospy.Publisher(self.lookup_plan,Path,queue_size=10)

        # set thread
        thread.start_new_thread(self.update_light,())
        thread.start_new_thread(self.update_status,())

        # run
        self.run()

    def callback(self,msg):
        # rospy.loginfo('callback')
        self.is_update = True
        # rospy.loginfo()
        self.plan = msg

    def update_status(self):

        update = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            update.sleep()

            self.prev_ts_state = self.ts_state
            self.get_status()

            if self.ts_state != self.prev_ts_state:
                rospy.loginfo('update status: %s'%(self.ts_state))

            self.driver.change_state(self.ts_state,0.00,0.00)

    def get_status(self):

        if self.stop_counter >= 5:
            self.ts_state = 'stop'

        if not self.is_update:
            self.stop_counter += 1
            return

        # update
        self.stop_counter = 0

        if self.angle >= self.turn_threshold:
            self.ts_state = 'left'
        elif self.angle <= -self.turn_threshold:
            self.ts_state = 'right'
        else:
            self.ts_state = 'top'

    def update_light(self):
        self.driver.set_rate(self.light_rate)

        self.driver.update_light()

    def run(self):
        rate = rospy.Rate(self.update_frequency)

        while not rospy.is_shutdown():

            rate.sleep()

            #check for update
            if not self.is_update :
                continue

            path = self.plan.poses
            if len(path)==0:
                continue

            lookup_plan_msg = Path()
            lookup_plan_msg.header.stamp = rospy.Time.now()
            lookup_plan_msg.header.frame_id = 'odom'


            self.is_update = False

            init = path[0].pose.orientation
            init_quan = (init.x,init.y,init.z,init.w)
            init_euler = tf.transformations.euler_from_quaternion(init_quan)
            init_np = np.array(init_euler)
            # init_np = np.array([-1,-1,-1])

            sum_df = np.array([0.00,0.00,0.00])

            for i in range(0,min(10,len(path) )):
                lookup_plan_msg.poses.append(path[i])
                quaternion = ( \
                    path[i].pose.orientation.x, \
                    path[i].pose.orientation.y, \
                    path[i].pose.orientation.z, \
                    path[i].pose.orientation.w)

                euler = tf.transformations.euler_from_quaternion(quaternion)
                np_euler = np.array(euler)
                df = np_euler - init_np
                df = (df+3*pi)%(2*pi)-pi

                sum_df+=df

            sum_df = sum_df/10
            init_np += sum_df

            angle = np.array([0.00,0.00,0.00]) #init.pose.orientation
            for i in range(0,min(self.lookup,len(path) )):
                lookup_plan_msg.poses.append(path[i])
                quaternion = ( \
                    path[i].pose.orientation.x, \
                    path[i].pose.orientation.y, \
                    path[i].pose.orientation.z, \
                    path[i].pose.orientation.w)

                euler = tf.transformations.euler_from_quaternion(quaternion)
                np_euler = np.array(euler)
                df = np_euler - init_np
                df = (df+3*pi)%(2*pi)-pi

                # rospy.loginfo(df)
                angle = angle*(1-self.ratio)+df*self.ratio

                # a = targetA - sourceA
                # a = (a + 180) % 360 - 180

            rospy.loginfo(angle)
            self.angle = angle[2]
            self.lookup_plan_pub.publish(lookup_plan_msg)




if __name__ == '__main__':
    ptd = path_to_direction()

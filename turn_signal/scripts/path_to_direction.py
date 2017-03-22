#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
import numpy as np
from math import pi
import thread
from include import turn_signal

class path_to_direction():
    def __init__(self):
        rospy.init_node('path_to_direction')

        # read param
        self.update_frequency = rospy.get_param('update_frequency',10)
        self.ratio = rospy.get_param('ratio',0.6)
        self.lookup = rospy.get_param('lookup',200)
        self.turn_threshold = rospy.get_param('turn_threshold',0.3)
        self.listen_topic = rospy.get_param('listen_topic','/move_base/TrajectoryPlannerROS/global_plan')
        # self.listen_topic = rospy.get_param('listen_topic','/move_base/DWAPlannerROS/global_plan')
        self.lookup_plan = rospy.get_param('lookup_plan','lookup_plan')

        # self.sub_topic = rospy.get_param('~sub_topic','/cmd_vel')

        self.dev = rospy.get_param('dev','/dev/arduino-nuke')
        self.baudrate = rospy.get_param('baudrate',115200)
        self.odom_topic = rospy.get_param('odom_topic','/odom')

        self.update_rate = rospy.get_param('~update_rate',5.0)
        self.light_rate = rospy.get_param('~light_rate',10.0)

        # init
        self.is_update = False
        #set pixel_driver
        try:
            self.driver = turn_signal.turn_signal(self.dev,self.baudrate)
            rospy.loginfo('driver workwell')
        except:
            rospy.loginfo('bad')
        self.is_stop = True

        # rospy.loginfo('work')
        self.ts_state = 'stop'
        self.angle = 0.00
        self.stop_counter = 0
        self.odom = Odometry()

        # listen
        rospy.Subscriber(self.listen_topic,Path,self.callback)
        # rospy.Subscriber(self.odom_topic,Odometry,self.odom_callback)
        rospy.Subscriber(self.odom_topic,PoseWithCovarianceStamped,self.odom_callback)

        # pubblish
        self.lookup_plan_pub = rospy.Publisher(self.lookup_plan,Path,queue_size=10)
        self.demo_pub = rospy.Publisher('demo_odom',Odometry,queue_size=10)
        # self.lookup_plan_pub = rospy.Publisher(self.lookup_plan,Path,queue_size=10)

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

        # rospy.loginfo(self.plan)

    def odom_callback(self,msg):
        self.odom = msg

    def update_status(self):

        update = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            update.sleep()

            self.prev_ts_state = self.ts_state
            self.get_status()
            # rospy.loginfo('status : {}, angle : {}'.format(self.ts_state,self.angle))

            if self.ts_state != self.prev_ts_state:
                rospy.loginfo('status : {}, angle : {}'.format(self.ts_state,self.angle))

            self.driver.change_state(self.ts_state,0.00,0.00)

    def get_status(self):

        if self.is_stop:
            self.ts_state = 'stop'
            return

        if self.stop_counter >= 10:
            self.ts_state = 'stop'

        # update
        self.stop_counter = 0


        # rospy.loginfo('angle : %f'%(self.angle))
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
                # rospy.loginfo(self.is_update)
                continue

            path = self.plan.poses
            if len(path)==0:
                continue

            lookup_plan_msg = Path()
            lookup_plan_msg.header.stamp = rospy.Time.now()
            lookup_plan_msg.header.frame_id = 'odom'


            # self.is_update = False

            # get order
            order = self.get_order()
            lookup_to = min(self.lookup,len(path)-order) + order

            if abs(lookup_to - order) <= 3:
                self.is_update = False
                self.is_stop = True
                # rospy.loginfo('what %d %d'%(order,lookup_to))
                continue
            else:
                self.is_stop = False

            init = self.odom.pose.pose.orientation
            init_quan = (init.x,init.y,init.z,init.w)
            init_euler = tf.transformations.euler_from_quaternion(init_quan)
            init_np = np.array(init_euler)
            # rospy.loginfo(init_np)

            robot_pose = self.odom.pose.pose.position
            robot_pose_np = np.array([robot_pose.x,robot_pose.y,robot_pose.z])

            angle = np.array([0.00,0.00,0.00]) #init.pose.orientation
            for i in range(order,lookup_to):
                lookup_plan_msg.poses.append(path[i])
                # quaternion = ( \
                #     path[i].pose.orientation.x, \
                #     path[i].pose.orientation.y, \
                #     path[i].pose.orientation.z, \
                #     path[i].pose.orientation.w)

                # euler = tf.transformations.euler_from_quaternion(quaternion)
                # np_euler = np.array(euler)

                l_pose = path[i].pose.position
                l_pose_np = np.array([l_pose.x,l_pose.y,l_pose.z])
                vec_np = l_pose_np - robot_pose_np

                np_euler = np.arctan2(vec_np[1],vec_np[0])
                np_euler = np.array([0,0,np_euler])
                # rospy.loginfo('np_euler : {}'.format(np_euler))

                df = np_euler - init_np
                df = (df+3*pi)%(2*pi)-pi

                # rospy.loginfo(df)
                angle = angle*(1-self.ratio)+df*self.ratio

                # a = targetA - sourceA
                # a = (a + 180) % 360 - 180

            # rospy.loginfo('Diff angle : %f'%(angle[2]))
            self.angle = angle[2]
            self.lookup_plan_pub.publish(lookup_plan_msg)

    def get_order(self):
        pose = self.odom.pose.pose.position
        np_pose = np.array([pose.x,pose.y,pose.z])

        poses = self.plan.poses
        closer = 100000.0
        order = 0

        debug =[]

        for i in range(0,len(poses)):
            i_pose = poses[i].pose.position
            np_i_pose = np.array([i_pose.x,i_pose.y,i_pose.z])

            dist = np.linalg.norm(np_pose - np_i_pose)
            # rospy.loginfo(dist)
            debug.append(dist)

            if dist < closer:
                closer = dist
                order = i
                # rospy.loginfo('%d dist %f'%(order,closer))

            px = Odometry()
            px.header.stamp = rospy.Time.now()
            px.header.frame_id = 'odom'

            px.pose.pose =  poses[i].pose
            # rospy.loginfo(px)
            self.demo_pub.publish(px)

        # if order == 0:
        #     rospy.loginfo(debug)
        px = Odometry()
        px.header.stamp = rospy.Time.now()
        px.header.frame_id = 'odom'

        px.pose.pose =  poses[order].pose
        # rospy.loginfo(px)
        self.demo_pub.publish(px)

        # rospy.loginfo('order : %d'%(order))
        return order


if __name__ == '__main__':
    ptd = path_to_direction()

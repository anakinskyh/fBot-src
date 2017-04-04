#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped,Pose
import tf
import numpy as np
from math import pi
import thread
from include import turn_signal as turn_signal
from include import turn_signal_C,turn_signal_E,turn_signal_D
import actionlib
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from actionlib_msgs.msg import GoalStatus,GoalStatusArray
from std_msgs.msg import Float64 as Float

class path_to_direction():
    def __init__(self):
        rospy.init_node('path_to_direction')

        # read param
        self.update_frequency = rospy.get_param('~update_frequency',10)
        self.ratio = rospy.get_param('~ratio',0.6)
        self.lookup = rospy.get_param('~lookup',200)
        self.turn_threshold = rospy.get_param('~turn_threshold',0.3)
        self.listen_topic = rospy.get_param('~listen_topic','/move_base/TrajectoryPlannerROS/global_plan')
        # self.listen_topic = rospy.get_param('listen_topic','/move_base/DWAPlannerROS/global_plan')
        self.lookup_plan = rospy.get_param('~lookup_plan','lookup_plan')
        self.move_base = rospy.get_param('~move_base','move_base')

        self.dev = rospy.get_param('~dev','/dev/arduino-nuke')
        self.baudrate = rospy.get_param('~baudrate',115200)
        self.odom_topic = rospy.get_param('~odom_topic','/raw_odom')

        self.update_rate = rospy.get_param('~update_rate',5.0)
        self.light_rate = rospy.get_param('~light_rate',5.0)

        self.type = rospy.get_param('~type','c')

        self.color = rospy.get_param('~color','yellow') # yellow green red
        self.ismove = rospy.get_param('~ismove',True)


        # init
        self.is_update = False
        self.move_base_state = 0
        self.pause_time = rospy.Time.now()

        #set pixel_driver
        try:
            if str.lower(self.type) == 'c':
                self.driver = turn_signal_C.turn_signal(self.dev,self.baudrate,color=self.color,ismove = self.ismove)
            elif str.lower(self.type) == 'd':
                self.driver = turn_signal_D.turn_signal(self.dev,self.baudrate,color=self.color,ismove = self.ismove)
            elif str.lower(self.type) == 'e':
                self.driver = turn_signal_E.turn_signal(self.dev,self.baudrate,color=self.color,ismove = self.ismove)
            elif str.lower(self.type) == 'a':
                self.driver = turn_signal.turn_signal(self.dev,self.baudrate,color=self.color,ismove = self.ismove)
            else:
                self.driver = turn_signal_C.turn_signal(self.dev,self.baudrate,color=self.color,ismove = self.ismove)

            rospy.loginfo('driver workwell')
        except:
            rospy.loginfo('bad')
        self.is_stop = True

        # rospy.loginfo('work')
        self.ts_state = 'stop'
        self.angle = 0.00
        self.stop_counter = 0
        self.odom = Odometry()

        # actionlib
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()

        # listen
        rospy.Subscriber(self.listen_topic,Path,self.callback)
        # rospy.Subscriber(self.odom_topic,PoseWithCovarianceStamped,self.odom_callback)
        rospy.Subscriber(self.odom_topic,Odometry,self.odom_callback)
        rospy.Subscriber('move_base/status',GoalStatusArray,self.status_callback)
        rospy.Subscriber('pause',Float,self.pause_callback)

        # pubblish
        self.lookup_plan_pub = rospy.Publisher(self.lookup_plan,Path,queue_size=10)
        self.demo_pub = rospy.Publisher('demo_odom',Odometry,queue_size=10)
        # self.lookup_plan_pub = rospy.Publisher(self.lookup_plan,Path,queue_size=10)

        # set thread
        thread.start_new_thread(self.update_light,())
        thread.start_new_thread(self.update_status,())
        thread.start_new_thread(self.update_tf,())

        # run
        self.run()

    def callback(self,msg):
        self.is_update = True
        self.plan = msg

    def pause_callback(self,msg):
        self.pause_time = rospy.Time.now() + rospy.Duration(msg.data)

    def odom_callback(self,msg):
        # rospy.logerr('recv : {}'.format(msg))
        # self.odom = msg
        x = 0

    def status_callback(self,msg):
        if len(msg.status_list) > 0:
            self.move_base_state = msg.status_list[len(msg.status_list)-1].status

    def update_tf(self):
        listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform("odom", "base_link", rospy.Time(0))
                # rospy.loginfo("{} {}".format(trans,rot) )

                msg = Odometry()
                msg.header.frame_id = 'odom'

                msg.pose.pose.position.x = trans[0]
                msg.pose.pose.position.y = trans[1]
                msg.pose.pose.position.z = trans[2]

                msg.pose.pose.orientation.x = rot[0]
                msg.pose.pose.orientation.y = rot[1]
                msg.pose.pose.orientation.z = rot[2]
                msg.pose.pose.orientation.w = rot[3]

                self.demo_pub.publish(msg)
                self.odom = msg
            except:
                rospy.loginfo('err')

    def update_status(self):

        update = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            update.sleep()

            self.prev_ts_state = self.ts_state
            self.get_status()
            # rospy.loginfo(self.ts_state)
            # rospy.loginfo('status : {}, angle : {}'.format(self.ts_state,self.angle))

            if self.ts_state != self.prev_ts_state:
                rospy.loginfo('status : {}, angle : {}'.format(self.ts_state,self.angle))
            # rospy.loginfo('status : {}, angle : {}'.format(self.ts_state,self.angle))
            self.driver.change_state(self.ts_state,0.00,0.00)

    def get_status(self):

        if rospy.Time.now() < self.pause_time:
            self.ts_state = 'stop'
            return

        # rospy.loginfo('{} {}'.format(self.move_base_state,GoalStatus.PENDING))
        if self.move_base_state == GoalStatus.PENDING \
            or self.move_base_state == GoalStatus.PREEMPTED \
            or self.move_base_state == GoalStatus.ABORTED \
            or self.move_base_state == GoalStatus.REJECTED \
            or self.move_base_state == GoalStatus.LOST \
            or self.move_base_state == GoalStatus.SUCCEEDED \
            :
            self.ts_state = 'stop'
            # rospy.loginfo('status {} {}'.format(self.move_base_state,GoalStatus.ABORTED))
            return

        # if self.stop_counter >= 10:
        #     self.ts_state = 'stop'
        #
        # # update
        # self.stop_counter = 0

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

            # rospy.loginfo('loop')

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

                l_pose = path[i].pose.position
                l_pose_np = np.array([l_pose.x,l_pose.y,l_pose.z])
                vec_np = l_pose_np - robot_pose_np

                np_euler = np.arctan2(vec_np[1],vec_np[0])
                np_euler = np.array([0,0,np_euler])
                # rospy.loginfo('np_euler : {}'.format(np_euler))

                df = np_euler - init_np
                df = (df+3*pi)%(2*pi)-pi

                # rospy.loginfo("df : {}".format(df) )
                angle = angle*(1-self.ratio)+df*self.ratio
            rospy.loginfo('status : {}, angle : {}'.format(self.ts_state,self.angle))
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
            # self.demo_pub.publish(px)

        # if order == 0:
        #     rospy.loginfo(debug)
        px = Odometry()
        px.header.stamp = rospy.Time.now()
        px.header.frame_id = 'odom'

        px.pose.pose =  poses[order].pose
        # rospy.loginfo(px)
        # self.demo_pub.publish(px)

        # rospy.loginfo('order : %d'%(order))
        return order


if __name__ == '__main__':
    ptd = path_to_direction()

#!/usr/bin/env python

import pickle
import rospy
from geometry_msgs.msg import Pose
from include import goal_setter
from actionlib_msgs.msg import GoalStatus

class command():
    def __init__(self,map =''):
        # if map=='':
        #     map = raw_input('What is map name?\n')
        #
        # self.point_name = 'map_points_%s'%(map)
        #
        #
        # try:
        #     self.map_points = pickle.load('%s.p'%(self.point_name),'rb')
        # except:
        #     rospy.loginfo('file not found')
        #     self.map_points = {'x':'y'}

        self.gs = goal_setter.goal_setter()
        # rospy.loginfo('hi')
        # self.run_command()
        # self.select_point()

    def run_command(self):
        menu ={'0':'select point to go'}
        while not rospy.is_shutdown():
            print 'What is your command?'
            for k,v in menu.iteritems():
                print '%s %s'%(k,v)

            sel = raw_input('command : ')

            if sel == '0':
                self.select_point()

    def select_point(self):
        # menu1 = {
        #     '0':['room1',[6.139, 4.375, 0.000,0.000, 0.000, 0.899, 0.438]],
        #     '1':['room2',[1.849, 6.881, 0.000,0.000, 0.000, 0.864, 0.504]],
        #     '2':['lift',[0.818, 9.167, 0.000,0.000, 0.000, -0.003, 1.000]]
        # }

        menu1 = {
            '0':['base',[-12.694, -4.987, 0.000,0.000, 0.000, 0.666, 0.746]],
            '1':['base2',[-16.540, -5.152, 0.000,0.000, 0.000, 0.658, 0.753]],
            '2':['computer',[-15.122, -9.882, 0.000, 0.000, 0.000, 0.672, 0.741]],
            '3':['lift1 bt',[-13.859, -13.907, 0.000,0.000, 0.000, 0.998, 0.060]],
            '4':['lift2 bt',[-14.328, -16.605, 0.000,0.000, 0.000, -0.063, 0.998]],
            '5':['lift3 bt',[-14.374, -19.442, 0.000,0.000, 0.000, 1.000, 0.028]],
            '6':['lift1 top',[-16.877, -13.537, 0.000,0.000, 0.000, 1.000, -0.007]],
            '7':['lift2 top',[-18.026, -16.187, 0.000,0.000, 0.000, -0.068, 0.998]],
            '8':['lift3 top',[-18.091, -19.304, 0.000,0.000, 0.000, 0.030, 1.000]],
            '9':['other lab',[-16.894, -23.731, 0.000,0.000, 0.000, 0.658, 0.753]]
        }

        self.menu1 = menu1

        menu2 = {
            'a':'insert point',
            'p':'path',
            's':'save',
            'g':'get status',
            'c':'get current goal',
            'e':'exit'
        }

        while not rospy.is_shutdown():
            print 'Where you wanna go?'
            for k,v in menu1.iteritems():
                # rospy.loginfo(menu1[i])
                print '%s\t%s'%(k,v[0])
            for k,v in menu2.iteritems():
                print '%s\t%s'%(k,v)

            sel = raw_input('command : ')

            if sel=='e':
                break
            elif sel=='s':
                print '%s.p'%(self.point_name)
                pickle.dump(self.map_points,open('%s.p'%(self.point_name),'wb'))
            elif sel=='a':
                name = raw_input('name : ')
                pose = raw_input('x y z x y z w : ')
                pose = str.split(pose)

            elif sel=='g':
                print self.gs.get_status()

            elif sel=='p':
                self.follow_path()

            else:
                arr_pose = menu1[sel][1]
                # pose.position
                pose = Pose()
                (pose.position.x,pose.position.y,pose.position.z) = arr_pose[0:3] #arr_pose[0:3]
                (pose.orientation.x,pose.orientation.y,pose.orientation.z, \
                    pose.orientation.w) = arr_pose[3:7]
                # rospy.loginfo('arr {}'.format(pose))
                self.gs.move_to(pose)

    def follow_path(self,timeout=10,qr_read = False):

        path1 = {
            '0':['all point','0','1','2','3','4','5','6','7','8','9','0'],
            '1':['lift-zig-zag','3','6','4','7','5','8'],
            '2':['testing','3','8','5','6','4','7']
        }

        menu2 = {
            'g':'get status',
            'e':'exit'
        }

        while not rospy.is_shutdown():
            print 'Path selection'

            for k,v in path1.iteritems():
                # rospy.loginfo(self.menu1[i])
                print '%s\t%s'%(k,v[0])
            for k,v in menu2.iteritems():
                print '%s\t%s'%(k,v)

            sel = raw_input('command : ')

            if sel=='e':
                break
            elif sel=='g':
                print self.gs.get_status()
            else:
                arr = path1[sel]
                # rospy.loginfo(arr)

                for i in range(1,len(arr)):
                    arr_pose = self.menu1[arr[i]][1]
                    rospy.loginfo('go to {}'.format(arr_pose))
                    pose = Pose()
                    (pose.position.x,pose.position.y,pose.position.z) = arr_pose[0:3] #arr_pose[0:3]
                    (pose.orientation.x,pose.orientation.y,pose.orientation.z, \
                        pose.orientation.w) = arr_pose[3:7]
                    self.gs.move_to(pose)
                    rospy.sleep(5)

                    last_time = rospy.Time.now()
                    wait_time = rospy.Duration(timeout)
                    while rospy.Time.now()  <= last_time +wait_time and self.gs.get_status() != GoalStatus.SUCCEEDED:
                        if self.gs.get_status() == GoalStatus.ACTIVE:
                            last_time = rospy.Time.now()

                    if self.gs.get_status() == GoalStatus.SUCCEEDED and qr_read:
                        rospy.loginfo('read qr')




if __name__ == '__main__':
    rospy.init_node('test')
    cm = command()

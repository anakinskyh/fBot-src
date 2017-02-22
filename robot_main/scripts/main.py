#!/usr/bin/env python

from include.goal_setter import goal_setter as gs
import thread
import rospy

class controller():

    def __init__(self):
        rospy.loginfo('run main')

    def get_param(self,move_base='move_base'):
        self.move_base = move_base

    def do(self):
        # do command from self.command
        rospy.loginfo('do')
        gs_ = gs()

        while not rospy.is_shutdown():
            # do command until ros is gone
            rospy.loginfo('run')
            if len(self.commands)==0:
                rospy.sleep(1)
                continue
            command = self.commands.pop(0)

            
            

    def run(self):
        # recieve command

        try:
            thread.start_new_thread(do,('Thread-1',2,) )
        except:
            rospy.loginfo('Log Error')

        while not rospy.is_shutdown():
            rospy.loginfo('run')
            command = input()

            # ![command] is override all command
            if (command[0]=='!'):
                self.commands = [command]
            else:
                self.commands.append(command)



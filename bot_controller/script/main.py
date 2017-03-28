#!/usr/bin/env python

import rospy
# from include import
import command

class main():
    """docstring for ."""
    def __init__(self):
        rospy.init_node('main')
        # rospy.loginfo('start main')

        self.command = command.command()
        self.main_menu()

    def main_menu(self):
        rospy.loginfo('main menu')
        main_menu = {
            '0':'go somewhere',
            '1':'setup record time',
            'e':'exit'
        }

        while not rospy.is_shutdown():
            print 'Main menu'
            for (k,v) in main_menu.iteritems():
                print ' %s\t%s'%(k,v)
            sel = raw_input('Command: ')
            sel = sel.strip()

            if sel == 'e':
                break
            elif sel == '0':
                self.command.select_point()
            elif sel == '1':
                rospy.loginfo('Unavailable')

if __name__ == '__main__':
    m = main()

#!/usr/bin/env python
# license removed for brevity
import rospy
import actionlib
# from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from optparse import OptionParser

import location

# location = {'x':[-1.051, 0.029, -3.093],
#             'y':[-0.962, -1.182, -3.141],
#             'z':[0.075, -2.476, -1.534],
#             'w':[0.179, 0.086, 3.131],
#             'start':[-0.970, 1.684, 1.562]}

# lab20_102
location = {'x':[-0.015, 0.060, 1.189],
            'y':[-1.669, -0.758, 2.697],
            'z':[-0.232, -2.517, -2.057],
            'w':[3.517, -2.358, -0.394]}


def talker():
    pub = rospy.Publisher('move_base_simple/goal',PoseStamped,queue_size=10)
    rospy.init_node('goal_sender',anonymous=True)

    # while not rospy.is_shutdown():
    #     line = raw_input()
    #     print line.'\n'

    while True:
        line = raw_input()
        # print line
        words = line.split()
        
        if words[0] == 'go':
            dest = words[1]
            if dest in location.keys():
                point = location[dest]
                pose = PoseStamped()

                pose.header.frame_id = 'map'
                pose.header.stamp = rospy.get_rostime()

                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                # pose.pose.position.z = point[2]

                pose.pose.orientation.w = 1

                pub.publish(pose)

        if words[0] == 'bye':
            break
            
if __name__ == '__main__':
    # try:
    #     talker()
    # except rospy.ROSInterruptException:
    #     pass
    talker()

import rospy
from std_msgs.msg import String

# listen to rech message and publish new goal
def callback(data):
    rospy.loginfo("")

def listener():
    rospy.init_node('set_goal')
    # rospy.Subscriber('',String,callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
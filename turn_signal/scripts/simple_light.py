import rospy
import serial
# from std_msgs.msg import String
# from geometry_msgs.msg import Twist

class light():

    def get_param(self):
        self.sub_topic = rospy.get_param('~sub_topic', '/cmd_vel')
        # self.use_light_ratio = rospy.get_param('~use_light_ratio', False)
        # self.light_ratio = rospy.get_param('~light_ratio', 0.0)
        # self.rate = rospy.get_param('~light_rate', 10.0)
        self.dev = rospy.get_param('~dev', '/dev/arduino')

    def init(self):
        rospy.init_node('simple_light')
        rospy.loginfo('start simple_light')
        self.get_param()
        while not rospy.is_shutdown():

            try:
                cmd = raw_input()
                cmd = input()

                if cmd == 'q':
                    break

                # print 'cat'
                cmd_atrs = cmd.split()
                ser_cmd = ','.join(cmd_atrs)
                ser_cmd = ser_cmd+'\n'
                print 'prepared ', ser_cmd

                self.ser = serial.Serial(self.dev, baudrate=9600, timeout=10)
                self.ser.write(ser_cmd)

                print 'send ', ser_cmd
            except :
                pass


if __name__ == '__main__':
    l = light()
    l.init()

#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
 
def publish_message():
    pub = rospy.Publisher('message_py', String, queue_size=10)
    rospy.init_node('simple_python_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "Hello Automatic Addison! %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
 
if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass
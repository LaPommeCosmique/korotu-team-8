
to install pigpio: https://abyz.me.uk/rpi/pigpio/download.html

wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install


NOTE: must start pigpio daemon
sudo pigpiod




Publisher node template

```
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

```


Subscriber node template

```
#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
     
def receive_message():
 
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'simple_python_subscriber' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('simple_python_subscriber', anonymous=True)
 
    rospy.Subscriber("message_py", String, callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
 
if __name__ == '__main__':
    receive_message()

```

IMPORTANT NOTE

```

###### NOTE: MAKE SURE YOU MAKE PYTHON FILE EXECUTABLE ------------------------------------------------------
###### chmod +x sensor_comms_node.py ----------------------------------------------------------------------------
###### or right click file -> properties -> make executable -------------------------------------------------

```


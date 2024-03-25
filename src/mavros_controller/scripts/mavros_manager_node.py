#!/usr/bin/env python3
import rospy
import time
import pigpio
from std_msgs.msg import Float32
from std_msgs.msg import UInt8

from mavros_msgs.srv import CommandBool
from geometry_msgs.msg import TwistStamped

###### NOTE: MAKE SURE YOU MAKE PYTHON FILE EXECUTABLE ------------------------------------------------------
###### chmod +x mavros_manager_node.py ----------------------------------------------------------------------------

max_horizontal_velocity = 0.5
max_vertical_velocity = 0.2
max_angular_velocity = 1

def manage_mavros():
    
    # Initialize rospy
    rospy.init_node('mavros_manager_node', anonymous=True)
    print("Starting mavros manager")

    # wait for service to become active
    rospy.wait_for_service('/mavros/cmd/arming')
    arming_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    print("Mavros has come online")

    # Publishers
    vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', TwistStamped, queue_size=10)

    # Flight parameters
    roll = 0
    pitch = 0
    throttle = 0
    yaw = 0

    rtl = False
    throttle_hold = False
    flight_mode = "STABILIZE" # can be STABILIZE, ALT_HOLD, LOITER
    # autonomous = False

    # arming
    armed = False
    arm_trigger_time = None # nanoseconds since epoch
    disarm_trigger_time = None
    arm_trigger_threshold = 0.2
    trigger_duration = 3_000_000_000 # 3e9 nanoseconds (3 seconds) 

    def arm():
        nonlocal armed
        print("Sending arm command")
        resp = arming_service(True)
        if resp.success:
            armed = True
            send_flight_parameters()
            print("Drone armed successfully!")
        else:
            print("Failed to arm the drone.")
    def disarm():
        nonlocal armed
        print("Sending arm command")
        resp = arming_service(False)
        if resp.success:
            armed = False
            print("Drone disarmed successfully!")
        else:
            print("Failed to disarm the drone.")
    
    def check_for_arming():
        if (armed is False):
            if (throttle < arm_trigger_threshold and yaw > (1 - arm_trigger_threshold)):
                if (arm_trigger_time is None):
                    arm_trigger_time = time.time_ns()
                elif (time.time_ns() - arm_trigger_time > trigger_duration):
                    arm()
                    arm_trigger_time = None
            else:
                arm_trigger_time = None
        else:
            arm_trigger_time = None

    def check_for_disarming():
        if (armed is True):
            if (throttle < arm_trigger_threshold and yaw < arm_trigger_threshold):
                if (disarm_trigger_time is None):
                    disarm_trigger_time = time.time_ns()
                elif (time.time_ns() - disarm_trigger_time > trigger_duration):
                    disarm()
                    disarm_trigger_time = None
            else:
                disarm_trigger_time = None
        else:
            disarm_trigger_time = None


    def send_flight_parameters():
        forward_velocity = (pitch * 2 - 1) * max_horizontal_velocity
        side_velocity = (roll * 2 - 1) * max_horizontal_velocity
        vertical_velocity = (throttle * 2 - 1) * max_vertical_velocity
        spin = (yaw * 2 - 1) * max_angular_velocity
            
        msg = TwistStamped()

        # Fill in the header of the message
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'  

        # Fill in the twist data
        msg.twist.linear.x = forward_velocity  # Linear velocity in x-direction
        msg.twist.linear.y = side_velocity  # Linear velocity in y-direction
        msg.twist.linear.z = vertical_velocity  # Linear velocity in z-direction
        # twist_stamped_msg.twist.angular.x = 0.0  # Angular velocity about x-axis
        # twist_stamped_msg.twist.angular.y = 0.0  # Angular velocity about y-axis
        msg.twist.angular.z = spin  # Angular velocity about z-axis

        vel_pub.publish(msg)

    # rc channel callbacks
    def rc_roll_callback(value):
        nonlocal roll
        roll = value
        if (armed):
            send_flight_parameters()

    def rc_pitch_callback(value):
        nonlocal pitch
        pitch = value
        if (armed):
            send_flight_parameters()

    def rc_throttle_callback(value):
        nonlocal throttle
        throttle = value
        if (armed):
            send_flight_parameters()
            check_for_disarming()
        else:
            check_for_arming()

    def rc_yaw_callback(value):
        nonlocal yaw
        yaw = value
        if (armed):
            send_flight_parameters()
            check_for_disarming()
        else:
            check_for_arming()

    # rc channel switch callbacks
    def rc_switch_a_callback(value):
        nonlocal rtl
        rtl = True if value is 1 else False
        if (armed):
            send_flight_parameters()

    def rc_switch_b_callback(value):
        nonlocal throttle_hold
        throttle_hold = True if value is 1 else False
        if (armed):
            send_flight_parameters()

    def rc_switch_c_callback(value):
        nonlocal flight_mode
        flight_mode = "LOITER" if value is 2 else ("ALT_HOLD" if value is 1 else "STABILIZE")
        if (armed):
            send_flight_parameters()

    def rc_switch_d_callback(value):
        pass

    # RC subscribers
    rospy.Subscriber('comms/rc/roll', rc_roll_callback)
    rospy.Subscriber('comms/rc/pitch', rc_pitch_callback)
    rospy.Subscriber('comms/rc/throttle', rc_throttle_callback)
    rospy.Subscriber('comms/rc/yaw', rc_yaw_callback)
    rospy.Subscriber('comms/rc/switch_a', rc_switch_a_callback)
    rospy.Subscriber('comms/rc/switch_b', rc_switch_b_callback)
    rospy.Subscriber('comms/rc/switch_c', rc_switch_c_callback)
    rospy.Subscriber('comms/rc/switch_d', rc_switch_d_callback)

    # listen to mavros state for arming

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        manage_mavros()
    except rospy.ROSInterruptException:
        pass

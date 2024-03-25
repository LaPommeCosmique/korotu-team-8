#!/usr/bin/env python3
import rospy
import time
import pigpio
from std_msgs.msg import Float32
from std_msgs.msg import UInt8

###### NOTE: MAKE SURE YOU MAKE PYTHON FILE EXECUTABLE ------------------------------------------------------
###### chmod +x rc_comms_node.py ----------------------------------------------------------------------------


def receive_rc():
    
    # Initialize rospy
    rospy.init_node('rc_comms_node', anonymous=True)

    # Initialize pi
    pi = pigpio.pi()

    # Check if pi initialization successful
    if not pi.connected:
       print("Error: pi not initialized")
       exit()
    else:
       print("Initialized to pi")

    # Signal PIN on PI
    rc_signal = 4

    # Track PPM signal information
    start_of_frame = False # flag that is triggered when we loop to the first PPM signal the very first time 
    channel = 0 # current PPM channel signal
    last_tick = None # micros since boot (wraps roughly every 72 minutes)

    # Track channel values
    channel_values = [None] * 8

    # Trigger values (only update channel value if more than threshold has changed)
    trigger_thresholds = [100] * 8

    # Publisher callback factories
    def get_roll_pub_callback():
        pub = rospy.Publisher('comms/rc/roll', Float32, queue_size=10)
        return lambda pwm_micros : pub.publish((pwm_micros - 1000) / 1000)
    def get_pitch_pub_callback():
        pub = rospy.Publisher('comms/rc/pitch', Float32, queue_size=10)
        return lambda pwm_micros : pub.publish((pwm_micros - 1000) / 1000)
    def get_throttle_pub_callback():
        pub = rospy.Publisher('comms/rc/throttle', Float32, queue_size=10)
        return lambda pwm_micros : pub.publish((pwm_micros - 1000) / 1000)
    def get_yaw_pub_callback():
        pub = rospy.Publisher('comms/rc/yaw', Float32, queue_size=10)
        return lambda pwm_micros : pub.publish((pwm_micros - 1000) / 1000)
    def get_swa_pub_callback(): # two states - 0, 1
        pub = rospy.Publisher('comms/rc/switch_a', UInt8, queue_size=10)
        return lambda pwm_micros : pub.publish(round((pwm_micros - 1000) / 1000))
    def get_swb_pub_callback(): # two states - 0, 1
        pub = rospy.Publisher('comms/rc/switch_b', UInt8, queue_size=10)
        return lambda pwm_micros : pub.publish(round((pwm_micros - 1000) / 1000))
    def get_swc_pub_callback(): # three states - 0, 1, 2
        pub = rospy.Publisher('comms/rc/switch_c', UInt8, queue_size=10)
        return lambda pwm_micros : pub.publish(round((pwm_micros - 1000) / 500))
    def get_swd_pub_callback(): # two states - 0, 1
        pub = rospy.Publisher('comms/rc/switch_d', UInt8, queue_size=10)
        return lambda pwm_micros : pub.publish(round((pwm_micros - 1000) / 1000))

    # Publisher callback functions
    publisher_callbacks = [get_roll_pub_callback(), get_pitch_pub_callback(),
                            get_throttle_pub_callback(), get_yaw_pub_callback(),
                            get_swc_pub_callback(), get_swd_pub_callback(),
                            get_swa_pub_callback(), get_swb_pub_callback()]


    # Callback function for rc signal edge change
    def on_edge_fall(gpio, level, tick):
       """
       Callback function that is triggered whenever the specified edge is detected

       Parameters
  	    gpio: GPIO pin number (0 to 31)
  	    level: type of edge (0 - FALLING_EDGE, 1 - RISING_EDGE, 2 - no change / timeout)
  	    tick: number of microseconds since boot
        	    WARNING: THIS WRAPS ROUGHLY EVERY 72 MINUTES
       """
              
       nonlocal start_of_frame, channel, last_tick

       if last_tick is not None:
          diff = pigpio.tickDiff(last_tick, tick)
          if diff > 3000: # start of frame
             start_of_frame = True
             channel = 0
          else:
             if start_of_frame:                

                # check if we need to update channel
                if(channel_values[channel] is None or (abs(channel_values[channel] - diff) > trigger_thresholds[channel])):
                    channel_values[channel] = diff
                    publisher_callbacks[channel](diff)

                channel += 1

       last_tick = tick

    pi.set_mode(rc_signal, pigpio.INPUT)

    on_edge_fall_callback = pi.callback(rc_signal, pigpio.FALLING_EDGE, on_edge_fall)
    rospy.loginfo("Listening to RC signals")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    # time.sleep(60)

    on_edge_fall_callback.cancel()

    pi.stop()


if __name__ == '__main__':
    try:
        receive_rc()
    except rospy.ROSInterruptException:
        pass

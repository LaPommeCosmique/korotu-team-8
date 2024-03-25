#!/usr/bin/env python3
import serial

import rospy
from std_msgs.msg import String
import time
import pigpio

from ....protocol.drone_side import Drone


"""
device
    init
        send_cb: callback - takes one argument for serialized data to send
        rec_cb: callback - no arguments, return received data [bytes]
        in_msg: protobuf message type to receive
        out_msg: protobuf message type to send
    service_messages: parse, decode messages
        timeout_ms: number of ms to attempt to resolve incomplete incoming messages
        returns:
            msg_list: list of all decoded messages
            fault: false when all messages are properly decoded
    send_keep_alive - shold be done once a second
    send_connection_termination - kill connection by sending STATUS_FAULT
    is_connection_active - return true if active connection, false otherwise



drone
    init
        send_cb - callback for sending data
        rec_cb - callback for receiving data
        get_pos_cb - return current drone position with no argument in form (long, lat)
        all_msgs_cbs - dict of received protobuf fields and associated callback functions
    
    connect

    send_drone_status - send current drone status (action, position, target waypoint)
        status - DRONE_STATUS
        current_long, current_lat - coordinates
        next_long, next_lat - next waypoint

    send_obstacle - send detected obstacle to sensor
        obstacle_pos_long, obstacle_pos_lat: coordiantes
        obstacle_est_size: estimated size



"""











current_waypoint = [None, None]
current_status = "OFF" # can be OFF, ON_ACTIVE, ON_INACTIVE, ON_FAULT

last_pos = [None, None]
last_vel = [None, None]
last_pos_time = None



last_gen_status_sent = None
last_drone_status_sent = None


device_name = '/dev/ttyACM0'

# create callback function for sending data through serial
def create_send_cb(ser):
    return lambda data : ser.write(data)

# create callback for extracting data from serial
def create_rec_cb(ser):
    return lambda _ : ser.read(ser.in_waiting)

# create callback for getting current drone position
def create_get_pos_cb():
    pass


def send_keep_alives(drone, rate):
    while current_status == "ON_ACTIVE" or current_status == "ON_INACTIVE":
        drone.send_drone_status()
        rate.sleep()

def manage_comms():

    ser = serial.Serial(device_name, 115200, timeout=1)
    ser.reset_input_buffer()
    
    # IO callback functions
    send_cb = create_send_cb(ser)
    rec_cb = create_rec_cb(ser)
    get_pos_cb = create_get_pos_cb()

    drone = Drone(send_cb, rec_cb, get_pos_cb, all_msgs_cb)
    
    # Initialize rospy
    rospy.init_node('rc_comms_node', anonymous=True)

    # If status is ON_ACTIVE or ON_INACTIVE, we should send keep_alive twice a second
    rate = rospy.Rate(2)  # 2hz

    # Create publishers
    general_status_pub = rospy.Publisher('comms/general_status', String, queue_size=10)
    drone_status_pub = rospy.Publisher('comms/drone_status', String, queue_size=10)
    waypoint_pub = rospy.Publisher('comms/waypoint', String, queue_size=10)

    # Create subscribers
    obstacle
    pose_sub = message_filters.Subscriber('/slam_out_pose', PoseStamped) 




    

    # states OFF, ON_FAULT, ON_ACTIVE, ON_INACTIVE

    # listen to arming
    

    # subscribe to swd
        # on - initiate connect
            # if OFF
                # if grounded
                    # set to ON_FAULT
                # else
                    # clear waypoint
                    # set to ON_INACTIVE
                    # start keep alives
        # off 
            # if ON_ACTIVE
                # stop drone (send waypoint null)
                # initiate disconnect
                # set OFF
            # if ON_INACTIVE
                # initiate disconnect
                # set OFF
            # if ON_FAULT
                # set OFF
            # if OFF
                # do nothing
        
    # subscribe to gen_status
        # if OK
            # if ON_ACTIVE, ON_INACTIVE
                # good, do nothing
            # if ON_FAULT
                # send fault/disconnect
            # if OFF
                # send fault/disconnect
        # if FAULT
            # if ON_ACTIVE
                # stop drone
                # set ON_FAULT
                # send fault/disconnect
            # ON_INACTIVE
                # set ON_FAULT
                # send fault/disconnect
            # if ON_FAULT
                # do nothing
            # if OFF
                # do nothing

    # subscribe to new waypoint
        # if ON_ACTIVE or ON_INACTIVE
            # publish to WAYPOINT
            # set to ON_ACTIVE
    
    # loop 0.5 sec
        # if ON_ACTIVE 
            # send MOVING_TO_WAYPOINT
        # if ON_INACTIVE
            # send HOVERING
        # if ON_FAULT, OFF
            # dont do anything

    # subscribe to current drone position
        # if ON_ACTIVE
            # if distance is < tolerance
                # clear Waypoint
                # switch to ON_INACTIVE
                # send HOVERING
    
    # subscribe to obstacle
        # if ON_ACTIVE
            # stop drone
            # send OBSTACLE
        # if ON_INACTIVE
            # send OBSTACLE
        # if ON_FAULT, OFF
            # do nothing
    
    # subscribe to user inputs
        # if ON_ACTIVE, ON_INACTIVE
            # if yaw, pitch, roll, thrust
                # stop drone
                # send FAULT
                # set ON_FAULT
    








if __name__ == '__main__':
    try:
        manage_comms()
    except rospy.ROSInterruptException:
        pass

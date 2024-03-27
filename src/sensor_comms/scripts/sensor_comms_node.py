#!/usr/bin/env python3
import serial

import rospy
from std_msgs.msg import String
import time
import pigpio

from ....protocol.drone_side import Drone
from ....protocol.output.interface_pb2 import DroneStatus

from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from std_msgs.msg import Header

from geometry_msgs.msg import PointStamped, Point32

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


device_name = '/dev/ttyACM0'

def manage_comms():

    # Initialize rospy
    rospy.init_node('rc_comms_node', anonymous=True)

    # Initialize serial
    ser = serial.Serial(device_name, 115200, timeout=1)
    ser.reset_input_buffer()
    
    # IO callback functions
    send_cb = lambda data : ser.write(data)
    rec_cb = lambda _ : ser.read(ser.in_waiting)

    # Comms status flag and publisher
    comms_status = "OFF" # can be OFF, ON_ACTIVE, ON_INACTIVE, ON_FAULT
    comms_status_pub = rospy.Publisher('comms/sensor/status', String, queue_size=10)

    # Update comms status
    def set_comms_status(status):
        nonlocal comms_status
        if (status == "OFF" or status == "ON_ACTIVE" or status == "ON_INACTIVE" or status == "ON_FAULT"):
            comms_status = status
            comms_status_pub.publish(comms_status)
    
    # Variables to determine precise location (long, lat)
    last_pos = None
    last_vel = None
    last_pos_update_time = None # nanoseconds since epoch

    # Get current drone position
    def get_current_pos():

        if (last_pos is None):
            return [None, None]
        if (last_vel is None or last_pos_update_time is None):
            return [last_pos[0], last_pos[1]]
        
        delta_time = 1.0 * (time.time_ns() - last_pos_update_time) / 10**9 # delta time in seconds

        current_pos = [last_pos[0] + last_vel[0] * delta_time, last_pos[1] + last_vel[1] * delta_time]
        return current_pos
    
    # Current waypoints (long, lat)
    current_waypoint = None
    
    # listen to sensor for waypoints. if received:
        # if ON_ACTIVE or ON_INACTIVE
            # publish to WAYPOINT
            # set to ON_ACTIVE

    # Waypoint publisher
    waypoint_pub = rospy.Publisher('/destination', PointStamped, queue_size=10)

    # Callback function for waypoints received from sensor
    def on_waypoint_receive(message):
        nonlocal current_waypoint

        long = message.new_waypoint.position_long
        lat = message.new_waypoint.position_lat

        if (comms_status == "ON_ACTIVE" or comms_status == "ON_INACTIVE"):
            if (long is not None and lat is not None):
                if (comms_status == "ON_INACTIVE"):
                    set_comms_status("ON_ACTIVE")
                
                current_waypoint = [long, lat]

                waypoint_msg = PointStamped()
                waypoint_msg.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        
                waypoint_msg.point.x = long
                waypoint_msg.point.y = lat
                waypoint_msg.point.z = 0

                waypoint_pub.publish(waypoint_msg)
            else:
                if (comms_status == "ON_ACTIVE"):
                    set_comms_status("ON_INACTIVE")
                current_waypoint = None
                # todo clear waypoint
                pass
    
    # create drone object
    drone = Drone(send_cb, rec_cb, get_current_pos, {"new_waypoint": on_waypoint_receive})

    # TODO todo send taring messages
    # TODO todo listen for waypoint arrival

    # listen to arming TODO todo
    # listen to drone elevation (?) TODO todo


    # subscribe to swd
        # on - initiate connect
            # if OFF
                # if grounded
                    # set to ON_FAULT
                # else
                    # clear waypoint (or not)
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
    

    def on_switch_d_input(switch_value):
        nonlocal current_waypoint
        # if switch is turned on, attempt to turn on autonomous mode
        if (switch_value.data == 1 and comms_status == "OFF"):
            if (True): # todo check if drone is not grounded
                # todo clear waypoint (or maybe not)
                success = drone.connect(1) # 1 second time out
                if (success == True):
                    current_waypoint = None
                    set_comms_status("ON_INACTIVE")
                else:
                    current_waypoint = None
                    set_comms_status("ON_FAULT")
                    drone.send_connection_termination()
        # if switch is turned off, turn off autonomous mode
        elif (switch_value.data == 0 and comms_status != "OFF"):
            if (comms_status == "ON_ACTIVE"):
                # todo clear waypoint
                pass
            if (comms_status == "ON_ACTIVE" or comms_status == "ON_INACTIVE"):
                drone.send_connection_termination()
            
            current_waypoint = None
            set_comms_status("OFF")

    # subscribe to d input
    rospy.Subscriber('/comms/rc/switch_d', UInt8, on_switch_d_input)

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
            # set ON_INACTIVE
        # if ON_INACTIVE
            # send OBSTACLE
        # if ON_FAULT, OFF
            # do nothing
    
    
    # callback function for when obstacle is detected
    def on_obstacle(obstacle_msg):
        nonlocal current_waypoint
        # if drone is following a waypoint, stop drone (out of scope?)
        if(comms_status == "ON_ACTIVE"):
            # todo stop drone
            current_waypoint = None
            set_comms_status("ON_INACTIVE")
            pass
        
        # check if we are in an active connection with drone
        if (comms_status == "ON_ACTIVE" or comms_status == "ON_INACTIVE"):
            long = obstacle_msg.x
            lat = obstacle_msg.y
            radius = obstacle_msg.z

            drone.send_obstacle(long, lat, radius)

    rospy.Subscriber('/obstacle', Point32, on_obstacle)

    # subscribe to user inputs
        # if ON_ACTIVE, ON_INACTIVE
            # if yaw, pitch, roll, thrust
                # stop drone
                # send FAULT
                # set ON_FAULT

    # callback function for when stick input is detected from user
    def on_stick_input(input):
        nonlocal current_waypoint
        # if drone is following a waypoint, clear waypoint (out of scope?)
        if(comms_status == "ON_ACTIVE"):
            # todo clear waypoint
            pass

        # if we are currently in ON_ACTIVE or ON_INACTIVE state, we stop autonomous operation by setting to ON_FAULT
        if (comms_status == "ON_ACTIVE" or comms_status == "ON_INACTIVE"):
            current_waypoint = None
            set_comms_status("ON_FAULT")
            drone.send_connection_termination()

    rospy.Subscriber('/rc/comms/yaw', Float32, on_stick_input)
    rospy.Subscriber('/rc/comms/throttle', Float32, on_stick_input)
    rospy.Subscriber('/rc/comms/pitch', Float32, on_stick_input)
    rospy.Subscriber('/rc/comms/roll', Float32, on_stick_input)

    # todo determine behaviour of switch inputs


    # loop 0.5 sec
        # if ON_ACTIVE 
            # send MOVING_TO_WAYPOINT
        # if ON_INACTIVE
            # send HOVERING
        # if ON_FAULT, OFF
            # dont do anything

    rate = rospy.Rate(2)  # 2 hz

    # If status is ON_ACTIVE or ON_INACTIVE, we should send keep_alive twice a second
    while not rospy.is_shutdown():
        drone.service_messages()

        # check if we are connected
        if (comms_status == "ON_ACTIVE" or comms_status == "ON_INACTIVE"):
            if (drone.is_connected()):
                # we are connected and communicating

                # determine drone status
                if (comms_status == "ON_ACTIVE"):
                    drone_status = DroneStatus.DRONE_STATUS_MOVING_TO_WAYPOINT
                if (comms_status == "ON_INACTIVE"):
                    drone_status = DroneStatus.DRONE_STATUS_HOVERING
                
                # determine current coordinates
                current_long, current_lat = get_current_pos()

                # determine next coordinates
                next_long = current_waypoint[0] if current_waypoint is not None else None
                next_lat = current_waypoint[1] if current_waypoint is not None else None

                # send status update to drone
                drone.send_drone_status(drone_status, current_long, current_lat, next_long, next_lat)                
            else:
                # sensor has disconnected, we are no longer communicating
                current_waypoint = None
                set_comms_status("ON_FAULT")
                drone.send_connection_termination()
        elif (comms_status == "ON_FAULT"):
            if (drone.is_connected()):
                # drone is attempting to connect even though we are faulted. termination connection
                drone.send_connection_termination()

        # sleep for 0.5s (2 hz)
        rate.sleep()




if __name__ == '__main__':
    try:
        manage_comms()
    except rospy.ROSInterruptException:
        pass

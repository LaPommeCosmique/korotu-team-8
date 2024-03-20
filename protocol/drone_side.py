"""
Drone side communication protocol API
"""
import output.interface_pb2 as I
import time as t
import transport_proto as transport
import device as Dev
from enum import Enum

# Enumerations for easy access of drone status
DRONE_STATUS_HOVERING = I.DRONE_STATUS_HOVERING
DRONE_STATUS_GROUNDED = I.DRONE_STATUS_GROUNDED
DRONE_STATUS_MOVING_TO_WAYPOINT = I.DRONE_STATUS_MOVING_TO_WAYPOINT


class Drone(Dev.Device):
    """
    Drone class, should be used on drone side to communicate with sensor

    User must register any desired protobuf field callbacks and remember to service messages periodically as well as send connection keep alives.
    Conection keep alives are ideally sent once per 500ms, along with servicing messages.

    Note that user callbacks using any coordinates grabbed directly from the sensor messages should remember to add
    self.long_delta and self.lat_delta since the drone and sensor do not necessarily use the same coordinate frames.
    """

    def __init__(self, send_cb, rec_cb, get_pos_cb, all_msg_cbs : dict) -> None:
        """
        Initialize the drone object

        Arguments:
            send_cb: IO layer communication callback to send data out (i.e serial) and should have one argument for the serialized data to send
            rec_cb: IO layer communication callback to receive data in (i.e serial) and should have no arguments and return the received data
            get_pos_cb: Return the current drone position with no argument in form (position_long, position_lat)
            all_msg_cbs: Dictionary of receive side protobuf fields and associated callback functions, use to add custom handler for receiving certain messages/events
        """
        msg_cbs = {'tare_position' : self._handle_tared_position_update}
        msg_cbs = {**msg_cbs, **all_msg_cbs}
        super().__init__(send_cb=send_cb, rec_cb=rec_cb, in_msg=I.ToDrone, out_msg=I.ToSensor, msg_callbacks=msg_cbs)
        self.long_delta = 0.0
        self.lat_delta = 0.0
        self.get_pos_cb = get_pos_cb


    def connect(self, timeout_s: int) -> bool:
        """
        Connects to sensor and clears any active sessions. Should only be called when connection is not already established

        Arguments:
            timeout_s: Number of seconds to attempt to connect to sensor
        Returns:
            is_connected: True if connection established, False otherwise
        """

        # First send the status OK message to set up connection
        self.faulted = False
        self.send_keep_alive()

        t_start = t.time()
        connected = False

        # Poll sensor until timeout for connection status as well as tared position
        # First send the status OK message to set up connection
        while ((t.time() - t_start) < timeout_s):
            self.service_messages(timeout_ms=100)
            connected = self.is_connection_active()
            if connected == True:
                break

        return connected


    def _handle_tared_position_update(self, msg) -> None:
        """
        Internal handler for updating the tared position.

        Arguments:
            msg: Incoming message containing the new tared position
        """
        drone_long, drone_lat = self.get_pos_cb()
        self.long_delta = msg.tare_position.position_long - drone_long
        self.lat_delta = msg.tare_position.position_lat - drone_lat


    def send_drone_status(self, status, current_long : float, current_lat : float, next_long=None, next_lat=None) -> None:
        """
        Send the current drone status (action, position, target waypoint) to the sensor. Should be sent at about 1s cycle time.

        Arguments:
            status: DRONE_STATUS of the current drone state
            current_long: current longitude of the drone (from the drone's perspective)
            current_lat: current latitude of the drone (from the drone's perspective)
            next_long: longitude of the waypoint the drone is currently moving toward
            next_lat: latitude of the waypoint the drone is currently moving toward
        """

        # Set global drone status
        msg = I.ToSensor(drone_status=status)

        # Set drone current position, add in deltas to square positions with sensor
        cur_pos = I.Waypoint(position_lat=(current_lat + self.lat_delta), position_long=(current_long + self.long_delta))
        msg.current_position.CopyFrom(cur_pos)

        # Set drone next position if available
        if next_long != None and next_lat != None:
            next_pos = I.Waypoint(position_lat=(next_lat + self.lat_delta), position_long=(next_long + self.long_delta))
            msg.next_position.CopyFrom(next_pos)

        # Send message
        msg_enc = transport.encode_msg(msg)
        self.send_cb(msg_enc)


    def send_obstacle(self, obstacle_pos_long : float, obstacle_pos_lat : float, obstacle_est_size : float) -> None:
        """
        Send the detection of an obstacle to the sensor.

        Arguments:
            obstacle_pos_long: Longitude of the detected obstacle
            obstacle_pos_lat: Latitude of the detected obstacle
            obstacle_est_size: Estimated size of the detected obstacle
        """

        # Prepare message
        msg = I.ToSensor()
        waypoint = I.Waypoint(position_lat=(obstacle_pos_lat + self.lat_delta), position_long=(obstacle_pos_long + self.long_delta))
        obstacle = I.Obstacle(radius_m=obstacle_est_size)
        obstacle.location.CopyFrom(waypoint)
        msg.obstacle.CopyFrom(obstacle)

        # Send message
        msg_enc = transport.encode_msg(msg)
        self.send_cb(msg_enc)

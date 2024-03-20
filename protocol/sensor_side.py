"""
Sensor side communication protocol API
"""

import output.interface_pb2 as I
import time as t
import transport_proto as transport
import device as Dev

class Sensor(Dev.Device):
    """
    Sensor class, should be used on sensor side to communicate with drone

    User must register any desired protobuf field callbacks and remember to service messages periodically as well as send connection keep alives.
    Conection keep alives are ideally sent once per 500ms, along with servicing messages.
    """


    def __init__(self, send_cb, rec_cb, msg_cbs=None) -> None:
        """
        Initialize the sensor object.

        Arguments:
            send_cb: IO layer communication callback to send data out (i.e serial) and should have one argument for the serialized data to send
            rec_cb: IO layer communication callback to receive data in (i.e serial) and should have no arguments and return the received data
            all_msg_cbs: Dictionary of receive side protobuf fields and associated callback functions, use to add custom handler for receiving certain messages/events
        """
        if msg_cbs == None:
            msg_cbs = dict()
        super().__init__(send_cb=send_cb, rec_cb=rec_cb, in_msg=I.ToSensor, out_msg=I.ToDrone, msg_callbacks=msg_cbs)


    def connect(self, timeout_s: int, tare_lat : float, tare_long : float) -> bool:
        """
        Connects to drone and clears any active session data. Should only be called when connection is not already established.

        Arguments:
            timeout_s: Number of seconds to attempt to connect to sensor
            tare_lat: Latitude of current estimated sensor position to send to drone
            tare_long: Longitude of current estimated sensor position to send to drone
        Returns:
            is_connected: True if connection established, False otherwise
        """

        # First send the status OK message to set up connection
        self.faulted = False
        self.send_keep_alive()

        t_start = t.time()
        connected = False

        # # Poll drone until timeout for connection status
        while ((t.time() - t_start) < timeout_s):
            self.service_messages(timeout_ms=100)
            connected = self.is_connection_active()
            if connected == True:
                break

        # If the connection was properly initialized, then tare the waypoint, if not then exit in failure
        if connected == False:
            return False

        # Setup tare lat and long
        self.send_tare_position(pos_lat=tare_lat, pos_long=tare_long)

        return True


    def send_update_waypoint(self, pos_lat : float, pos_long : float) -> None:
        """
        Update the current waypoint for the drone.

        Note: The drone should immediately start moving to this waypoint once it is ready.

        Arguments:
            pos_lat: Latitude of the waypoint position
            pos_long: Longitude of the waypoint position
        """
        msg = I.ToDrone()
        new_pos = I.Waypoint(position_lat=pos_lat, position_long=pos_long)
        msg.new_waypoint.CopyFrom(new_pos)
        msg_enc = transport.encode_msg(msg)
        self.send_cb(msg_enc)


    def send_tare_position(self, pos_lat : float, pos_long : float) -> None:
        """
        Send a tare position to the drone.

        Note: Since the tare adjusts the relative offset of the drone-sensor coordinate system, any active waypoints should be
        resent to the drone after calling this in order for it to propely compute the required offsets.
        Arguments:
            pos_lat: Latitude of the tare position
            pos_long: Longitude of the tare position
        """
        # Setup tare lat and long
        msg = I.ToDrone()
        tare_pos = I.Waypoint(position_long=pos_long, position_lat=pos_lat)
        msg.tare_position.CopyFrom(tare_pos)
        msg_enc = transport.encode_msg(msg)
        self.send_cb(msg_enc)


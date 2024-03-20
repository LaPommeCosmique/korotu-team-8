"""
Generic device class

Common attributes between both drone and sensor
"""
import output.interface_pb2 as I
import time as t
import transport_proto as transport

CONNECTION_TIMEOUT_S = 1

class Device:
    """
    Generic device class

    Device contains connection parameters that are shared between both drone and sensor
    """

    def __init__(self, send_cb, rec_cb, in_msg, out_msg, msg_callbacks) -> None:
        """
        Initialize the device API object

        Arguments:
            send_cb: IO layer communication callback to send data out (i.e serial) and should have one argument for the serialized data to send
            rec_cb: IO layer communication callback to receive data in (i.e serial) and should have no arguments and return the received data
            in_msg: Protobuf message type to receive
            out_msg: Protobuf message type to send
            msg_callbacks: dictionary of receive side protobuf fields and associated callback functions, use to add custom handler for receiving certain messages/events
        """
        self.send_cb = send_cb
        self.rec_cb = rec_cb
        self.last_keep_alive = None
        self.faulted = True
        self.in_msg = in_msg
        self.out_msg = out_msg
        default_msg_callbacks = {'gen_status' : self._service_gen_status}
        self.msg_callbacks = {**default_msg_callbacks, **msg_callbacks}

    def _service_gen_status(self, msg) -> None:
        """
        Internal callback for general status message. Terminate/continue connection as needed

        Arguments:
            msg: message to parse
        """
        if msg.gen_status == I.STATUS_FAULT:
            self.last_keep_alive = None
            self.faulted = True
        elif msg.gen_status == I.STATUS_OK and self.faulted == False:
            self.last_keep_alive = t.time()


    def service_messages(self, timeout_ms : float) -> tuple:
        """
        Parse and decode incoming messages.

        If any incomplete messages are received, attempt to resolve the rest of the message for timeout_ms.
        timeout_ms should be set to something small (> 100ms). If a message is not able to be decoded, set fault as True.

        Arguments:
            timeout_ms: Number of ms to attempt to resolve incomplete incoming messages
        Returns:
            msg_list: List of all decoded messages received, if fault is set to True, last item will contain raw, undecoded partial message
            fault: True when all messages are properly decoded, false otherwise
        """
        timeout_s = timeout_ms * 0.001
        fault = False

        # Receive all buffered data
        b_rec = self.rec_cb()
        all_msgs, decode_complete, corrupted = transport.split_msg(b_rec, self.in_msg)
        n_good_msgs = len(all_msgs)

        if len(corrupted) > 0:
            fault = True
            all_msgs = all_msgs + corrupted
        elif decode_complete == False:
            # If we get an incomplete message, try and wait a bit to see if we get the rest of the data
            t_start = t.time()
            undecoded_fragment = all_msgs[-1]
            all_msgs.pop(-1)
            fault = True

            # Try and reconstruct last incomplete message
            while (t.time() - t_start) < timeout_s:
                new_msgs = self.rec_cb()
                if len(new_msgs) == 0:
                    continue
                new_total_buffer = undecoded_fragment + new_msgs
                new_msgs, decode_complete, corrupted = transport.split_msg(new_total_buffer, self.in_msg)

                if decode_complete == True:
                    # If all messages decoded, return with no fault
                    all_msgs = all_msgs + new_msgs
                    n_good_msgs = len(all_msgs)
                    fault = False
                    break
                else:
                    # If messages need still be decoded, keep trying until timeout
                    if len(new_msgs) > 1:
                        # If we are trying to receive a new fragment, get all the whole decoded messages and keep trying
                        all_msgs = all_msgs + new_msgs[0:-1]
                        n_good_msgs = len(all_msgs)
                        undecoded_fragment = new_msgs[-1]
                    else:
                        # If we are trying to receive the same fragment as before, keep trying
                        undecoded_fragment += new_msgs[-1]

        # Dispatch callbacks for any relevant received message
        msg_callback_keys = self.msg_callbacks.keys()
        for msg in all_msgs[0:n_good_msgs]:
            for key in msg_callback_keys:
                if msg.HasField(key):
                    self.msg_callbacks[key](msg)

        return all_msgs, fault


    def send_keep_alive(self) -> None:
        """
        Send the connection keep alive. Should be done at least once every second.
        """
        msg = self.out_msg()
        status = I.STATUS_OK
        msg.gen_status = status
        msg_enc = transport.encode_msg(msg)
        self.send_cb(msg_enc)


    def send_connection_termination(self) -> None:
        """
        Kill connection by sending STATUS_FAULT.

        Should be used only to unnexpectedly kill connection.

        Note: Device should wait for at least 1s before trying to re-initialize any connection.
        """
        # Send msgs
        msg = self.out_msg()
        status = I.STATUS_FAULT
        msg.gen_status = status
        msg_enc = transport.encode_msg(msg)
        self.send_cb(msg_enc)

        # Autokill connection on our side
        self.last_keep_alive = None
        self.faulted = True


    def is_connection_active(self) -> bool:
        """
        Check if there is a current active connection.

        Returns True if there is a connection and False otherwise.
        """
        if self.last_keep_alive == None:
            return False
        elif (t.time() - self.last_keep_alive) > CONNECTION_TIMEOUT_S:
            return False
        else:
            return True

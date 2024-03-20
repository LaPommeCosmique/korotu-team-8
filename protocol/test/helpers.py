"""
Helpers for Pytest based sensor-api tests
"""

import sensor_side as S
import output.interface_pb2 as I
import transport_proto as T
import threading
import time

def helper_send_message(msg, file) -> None:
    """
    Simple test helper for encoding and sending a message via file

    Message should be already assembled upon input
    """
    msg_bytes = msg.SerializeToString()
    length = (len(msg_bytes)).to_bytes(T.MESSAGE_LEN_SIZE_BYTES, T.MESSAGE_LEN_BYTE_ORDER)
    file.write(length)
    file.write(msg_bytes)
    file.flush()


def _helper_send_after_seconds(msg, file, seconds) -> None:
    """
    Runner function for async encode-send messages
    """
    time.sleep(seconds)
    helper_send_message(msg=msg, file=file)


def helper_send_message_after_seconds(msg, file, seconds) -> None:
    """
    Asynchronously encodes and sends data to file after a specified number of seconds
    """
    new_thread = threading.Thread(target=_helper_send_after_seconds, args=(msg, file, seconds,))
    new_thread.start()


def _helper_send_data_after_seconds(data, file, seconds) -> None:
    """
    Runner function for async sending data
    """
    time.sleep(seconds)
    file.write(data)
    file.flush()


def helper_send_data_after_seconds(data, file, seconds) -> None:
    """
    Asynchronously send raw data to file after a specified number of seconds
    """
    new_thread = threading.Thread(target=_helper_send_data_after_seconds, args=(data, file, seconds,))
    new_thread.start()


def helper_encode_message(msg) -> bytes:
    """
    Helper to just do the encoding part of the message
    """
    msg_bytes = msg.SerializeToString()
    length = (len(msg_bytes)).to_bytes(T.MESSAGE_LEN_SIZE_BYTES, T.MESSAGE_LEN_BYTE_ORDER)
    return length + msg_bytes


def helper_read_and_decode_first_whole_message(file_out, file_in, msg_type):
    """
    Read and decode exactly one whole message (if possible)

    file_out should be the file handle originally used to WRITE the data
    file_in should be the file handle to read from
    msg should be the expected message type to receive
    """
    file_out.flush()

    size_bytes = file_in.read(T.MESSAGE_LEN_SIZE_BYTES)

    if len(size_bytes) < T.MESSAGE_LEN_SIZE_BYTES:
        return None

    size = size_bytes[0 : T.MESSAGE_LEN_SIZE_BYTES]
    size = int.from_bytes(size, T.MESSAGE_LEN_BYTE_ORDER)

    data_bytes = file_in.read(size)
    msg = msg_type()
    msg.ParseFromString(data_bytes)
    return msg
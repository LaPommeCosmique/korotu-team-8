"""
Transport layer protocol helpers
"""
import output.interface_pb2 as I
from google.protobuf import message

MESSAGE_LEN_SIZE_BYTES = 4
MESSAGE_LEN_BYTE_ORDER = 'big'


def encode_msg(msg: message) -> bytes:
    """
    Encode messages to bytes in formate <len><message>.

    Arguments:
        msg: Protobuf message to encode
    """
    msg_bytes = msg.SerializeToString()
    len_bytes = (len(msg_bytes)).to_bytes(MESSAGE_LEN_SIZE_BYTES, MESSAGE_LEN_BYTE_ORDER)
    combined_msg = len_bytes + msg_bytes
    return combined_msg


def split_msg(bytes_in: bytes, decode_as: message) -> tuple:
    """
    Split and decode messages into a list.

    Arguments:
        bytes_in: Data to be decoded
        decode_as: Protobuf message formatting to use for decode
    Returns:
        messages: List of decoded messages, when decode_complete is set to False, the last item of the list will contain the raw bytes of the undecoded message
        decode_complete: If True, all messages properly decoded, if False, last message contains decoding failure
        corrupted_msg: List containing (assumed) complete but unparsable corrupted messages
    """

    msgs_out = []
    decode_complete = True
    corrupted = []

    while len(bytes_in) > MESSAGE_LEN_SIZE_BYTES:

        # Decode size - Verify that the whole size value is present in the buffer first
        if len(bytes_in) >= MESSAGE_LEN_SIZE_BYTES:
            size_b = bytes_in[0:MESSAGE_LEN_SIZE_BYTES]
            size = int.from_bytes(size_b, MESSAGE_LEN_BYTE_ORDER)
        else:
            decode_complete = False
            msgs_out.append(bytes_in)
            break

        # Decode message - Verify that the whole message value is present in the buffer first
        if len(bytes_in) >= (MESSAGE_LEN_SIZE_BYTES + size):
            msg = bytes_in[MESSAGE_LEN_SIZE_BYTES : size + MESSAGE_LEN_SIZE_BYTES]
            bytes_in = bytes_in[MESSAGE_LEN_SIZE_BYTES + size ::]
            # Parse protobuf
            d = decode_as()
            try:
                d.ParseFromString(msg)
                msgs_out.append(d)
            except:
                # Message corruption detected, save message for future evaluation
                corrupted.append(msg)

        else:
            decode_complete = False
            msgs_out.append(bytes_in)
            break

    return msgs_out, decode_complete, corrupted

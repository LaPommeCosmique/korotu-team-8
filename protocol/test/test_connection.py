"""
Test basic connection parameters

Since these are shared parameters, we do this with the sensor only
"""
import sensor_side as S
import output.interface_pb2 as I
import transport_proto as T
import pytest
import helpers
import time

TEST_INPUT_FILE = "in.txt"
TEST_OUTPUT_FILE = "out.txt"


### Mock callbacks

MSG_CALLBACK_DATA = None

def mock_message_callback(msg):
    """
    Set mock message callback data
    """
    global MSG_CALLBACK_DATA
    MSG_CALLBACK_DATA = msg


### Fixtures

@pytest.fixture
def sensor_config():
    """
    Setup and teardown basic sensor class using files IO for testing
    """

    # Test setup

    # Opening test files with "w" mode should overwrite them
    out_file_out = open(TEST_OUTPUT_FILE, 'wb')
    in_file_out = open(TEST_INPUT_FILE, 'wb')
    in_file_in = open(TEST_INPUT_FILE, 'rb')
    out_file_in = open(TEST_OUTPUT_FILE, 'rb')

    sensor = S.Sensor(send_cb=out_file_out.write, rec_cb=in_file_in.read)
    yield sensor, in_file_out, out_file_in, out_file_out, in_file_in

    # Test teardown

    in_file_in.close()
    out_file_out.close()
    in_file_in.close()
    out_file_in.close()


@pytest.fixture
def sensor_config_with_callback():
    """
    Setup and teardown basic sensor class using files IO for testing and register callback
    """

    # Test setup

    # Opening test files with "w" mode should overwrite them
    out_file_out = open(TEST_OUTPUT_FILE, 'wb')
    in_file_out = open(TEST_INPUT_FILE, 'wb')
    in_file_in = open(TEST_INPUT_FILE, 'rb')
    out_file_in = open(TEST_OUTPUT_FILE, 'rb')

    global MSG_CALLBACK_DATA
    MSG_CALLBACK_DATA = None

    # Create sensor with mock message callback
    sensor = S.Sensor(send_cb=out_file_out.write, rec_cb=in_file_in.read, msg_cbs={'obstacle' : mock_message_callback})
    yield sensor, in_file_out, out_file_in, out_file_out, in_file_in

    # Test teardown

    MSG_CALLBACK_DATA = None

    in_file_in.close()
    out_file_out.close()
    in_file_in.close()
    out_file_in.close()


### Connection setup and termination tests

def test_connection_no_response(sensor_config):
    """
    Test connection attempt when there is no response from Drone side
    """
    sensor, in_file_out, out_file_in, out_file_out, in_file_in = sensor_config

    # Check that not responding drone cannot be connected to
    assert sensor.connect(timeout_s=1, tare_lat=0., tare_long=0.) == False

    # Read output buffer to ensure that data was sent in the correct formatting
    msg = helpers.helper_read_and_decode_first_whole_message(out_file_out, out_file_in, I.ToDrone)
    assert msg.gen_status == I.STATUS_OK

    # Make sure the keep alive was the only message sent in this case
    msg = helpers.helper_read_and_decode_first_whole_message(out_file_out, out_file_in, I.ToDrone)
    assert msg == None

    assert sensor.is_connection_active() == False


def test_connection_timeout(sensor_config):
    """
    Test setting up a connection and letting it timeout
    """
    sensor, in_file_out, out_file_in, out_file_out, in_file_in = sensor_config

    # Put response into buffer first
    msg = I.ToSensor(gen_status=I.STATUS_OK)
    helpers.helper_send_message(msg, in_file_out)

    # Check that connection is properly initialized
    assert sensor.connect(timeout_s=1, tare_lat=0.1, tare_long=0.2) == True
    assert sensor.is_connection_active() == True

    # Sleep for 1 second - time connection out
    time.sleep(1)

    assert sensor.is_connection_active() == False


def test_connection_keep_alive(sensor_config):
    """
    Test setting up a connection and keeping it alive
    """
    sensor, in_file_out, out_file_in, out_file_out, in_file_in = sensor_config

    # Put response into buffer first
    msg = I.ToSensor(gen_status=I.STATUS_OK)
    helpers.helper_send_message(msg, in_file_out)

    # Check that connection is properly initialized
    assert sensor.connect(timeout_s=1, tare_lat=0.1, tare_long=0.2) == True
    assert sensor.is_connection_active() == True

    # Sleep for less than 1 second, connection should still be alive
    time.sleep(0.7)

    # Send msg
    msg = I.ToSensor(gen_status=I.STATUS_OK)
    helpers.helper_send_message(msg, in_file_out)

    # Read message
    sensor.service_messages(timeout_ms=100)

    # At this point, if the keep alive does not work, the connection should no longer be active
    time.sleep(0.7)
    assert sensor.is_connection_active() == True

    # Make sure connection can still timeout after sending repeated keep alives
    time.sleep(1)
    assert sensor.is_connection_active() == False


def test_connection_hard_disconnect(sensor_config):
    """
    Test setting up a connection and hard disconnecting it
    """
    sensor, in_file_out, out_file_in, out_file_out, in_file_in = sensor_config

    # Put response into buffer first
    msg = I.ToSensor(gen_status=I.STATUS_OK)
    helpers.helper_send_message(msg, in_file_out)

    # Check that connection is properly initialized
    assert sensor.connect(timeout_s=1, tare_lat=0.1, tare_long=0.2) == True
    assert sensor.is_connection_active() == True

    # Sleep for less than 1 second, connection should be alive
    time.sleep(0.1)
    assert sensor.is_connection_active() == True

    # Send fault message to sensor, should immediately kill connection
    msg = I.ToSensor(gen_status=I.STATUS_FAULT)
    helpers.helper_send_message(msg, in_file_out)
    sensor.service_messages(timeout_ms=100)
    assert sensor.is_connection_active() == False


def test_connection_hard_disconnect_and_keep_alive(sensor_config):
    """
    Test setting up a connection and hard disconnecting it
    """
    sensor, in_file_out, out_file_in, out_file_out, in_file_in = sensor_config

    # Put response into buffer first
    msg = I.ToSensor(gen_status=I.STATUS_OK)
    helpers.helper_send_message(msg, in_file_out)

    # Check that connection is properly initialized
    assert sensor.connect(timeout_s=1, tare_lat=0.1, tare_long=0.2) == True
    assert sensor.is_connection_active() == True

    # Sleep for less than 1 second, connection should be alive
    time.sleep(0.1)
    assert sensor.is_connection_active() == True

    # Send fault message to sensor, should immediately kill connection
    msg = I.ToSensor(gen_status=I.STATUS_FAULT)
    helpers.helper_send_message(msg, in_file_out)

    # Make sure that sending a keep alive OK after the fault does not reopen connection
    msg = I.ToSensor(gen_status=I.STATUS_OK)
    helpers.helper_send_message(msg, in_file_out)

    sensor.service_messages(timeout_ms=100)
    assert sensor.is_connection_active() == False


def test_connection_with_response(sensor_config):
    """
    Test setting up connection with a response
    """

    sensor, in_file_out, out_file_in, out_file_out, in_file_in = sensor_config

    # Schedule keep-alive to sensor at time which should initialize connection
    msg = I.ToSensor(gen_status=I.STATUS_OK)
    helpers.helper_send_message_after_seconds(msg=msg, file=in_file_out, seconds=0.3)

    # Check that connection is properly initialized
    assert sensor.connect(timeout_s=1, tare_lat=0.1, tare_long=0.2) == True
    assert sensor.is_connection_active() == True

    # First message should contain the keep alive
    msg = helpers.helper_read_and_decode_first_whole_message(out_file_out, out_file_in, I.ToDrone)
    assert msg.gen_status == I.STATUS_OK

    # Second message should contain the positional tare
    msg = helpers.helper_read_and_decode_first_whole_message(out_file_out, out_file_in, I.ToDrone)
    assert msg.tare_position.position_long == 0.2
    assert msg.tare_position.position_lat == 0.1

    # There should be no third message
    msg = helpers.helper_read_and_decode_first_whole_message(out_file_out, out_file_in, I.ToDrone)
    assert msg == None


def test_connection_send_keep_alive(sensor_config):
    sensor, in_file_out, out_file_in, out_file_out, in_file_in = sensor_config

    # Schedule keep-alive to sensor at time which should initialize connection
    msg = I.ToSensor(gen_status=I.STATUS_OK)
    helpers.helper_send_message_after_seconds(msg=msg, file=in_file_out, seconds=0.3)

    # Check that connection is properly initialized, clear all output messages
    assert sensor.connect(timeout_s=1, tare_lat=0.1, tare_long=0.2) == True
    assert sensor.is_connection_active() == True
    msg = helpers.helper_read_and_decode_first_whole_message(out_file_out, out_file_in, I.ToDrone)
    msg = helpers.helper_read_and_decode_first_whole_message(out_file_out, out_file_in, I.ToDrone)

    # Send keep alive
    sensor.send_keep_alive()

    # Parse keep alive response
    msg = helpers.helper_read_and_decode_first_whole_message(out_file_out, out_file_in, I.ToDrone)
    assert msg.gen_status == I.STATUS_OK


### Raw msg parsing tests

def test_service_msg(sensor_config):
    """
    Test sensor servicing single message from drone
    """
    sensor, in_file_out, out_file_in, out_file_out, in_file_in = sensor_config

    # Connect sensor and drone
    msg = I.ToSensor(gen_status=I.STATUS_OK)
    helpers.helper_send_message(msg, in_file_out)

    # Check that connection is properly initialized
    assert sensor.connect(timeout_s=1, tare_lat=0.1, tare_long=0.2) == True
    assert sensor.is_connection_active() == True

    msg = I.ToSensor(drone_status=I.DRONE_STATUS_GROUNDED)
    helpers.helper_send_message(msg, in_file_out)

    # Process single incoming message
    msg, fault = sensor.service_messages(timeout_ms=100)
    assert fault == False
    assert len(msg) == 1
    assert msg[0].drone_status == I.DRONE_STATUS_GROUNDED


def test_service_multimsg(sensor_config):
    """
    Test sensor servicing multiple message from drone
    """
    sensor, in_file_out, out_file_in, out_file_out, in_file_in = sensor_config

    # Connect sensor and drone
    msg = I.ToSensor(gen_status=I.STATUS_OK)
    helpers.helper_send_message(msg, in_file_out)

    # Check that connection is properly initialized
    assert sensor.connect(timeout_s=1, tare_lat=0.1, tare_long=0.2) == True
    assert sensor.is_connection_active() == True

    msg = I.ToSensor(drone_status=I.DRONE_STATUS_GROUNDED)
    helpers.helper_send_message(msg, in_file_out)

    msg = I.ToSensor(drone_status=I.DRONE_STATUS_HOVERING)
    helpers.helper_send_message(msg, in_file_out)

    msg = I.ToSensor(drone_status=I.DRONE_STATUS_MOVING_TO_WAYPOINT)
    helpers.helper_send_message(msg, in_file_out)

    # Process all incoming messages, make sure that order is preserved
    msgs, fault = sensor.service_messages(timeout_ms=100)
    assert fault == False
    assert len(msgs) == 3
    assert msgs[0].drone_status == I.DRONE_STATUS_GROUNDED
    assert msgs[1].drone_status == I.DRONE_STATUS_HOVERING
    assert msgs[2].drone_status == I.DRONE_STATUS_MOVING_TO_WAYPOINT


def test_service_half_msg(sensor_config):
    """
    Test sensor servicing a half message from the drone

    Should detect a message fault
    """
    sensor, in_file_out, out_file_in, out_file_out, in_file_in = sensor_config

    # Connect sensor and drone
    msg = I.ToSensor(gen_status=I.STATUS_OK)
    helpers.helper_send_message(msg, in_file_out)

    # Check that connection is properly initialized
    assert sensor.connect(timeout_s=1, tare_lat=0.1, tare_long=0.2) == True
    assert sensor.is_connection_active() == True

    # Send a complete message minus its last byte
    msg = I.ToSensor()
    waypoint = I.Waypoint(position_long=0.123, position_lat=-123.456)
    msg.current_position.CopyFrom(waypoint)
    msg_enc = helpers.helper_encode_message(msg)
    in_file_out.write(msg_enc[0 : len(msg_enc) - 1])
    in_file_out.flush()

    # Check that a message fault is detected
    msgs, fault = sensor.service_messages(timeout_ms=100)
    assert fault == True


def test_service_half_msg_complete(sensor_config):
    """
    Test sensor servicing a message sent in "2 halves"
    """
    sensor, in_file_out, out_file_in, out_file_out, in_file_in = sensor_config

    # Connect sensor and drone
    msg = I.ToSensor(gen_status=I.STATUS_OK)
    helpers.helper_send_message(msg, in_file_out)

    # Check that connection is properly initialized
    assert sensor.connect(timeout_s=1, tare_lat=0.1, tare_long=0.2) == True
    assert sensor.is_connection_active() == True

    # Send a complete message minus its last byte
    msg = I.ToSensor()
    waypoint = I.Waypoint(position_long=0.123, position_lat=-123.456)
    msg.current_position.CopyFrom(waypoint)
    msg_enc = helpers.helper_encode_message(msg)
    in_file_out.write(msg_enc[0 : len(msg_enc) - 1])
    in_file_out.flush()

    # Send last byte asynchronously 0.5 seconds from now
    helpers.helper_send_data_after_seconds(msg_enc[len(msg_enc) - 1 : len(msg_enc)], in_file_out, 0.5)

    # Since whole message has been properly received during the specified timeout period, we should properly reconstruct it
    msgs, fault = sensor.service_messages(timeout_ms=1000)
    assert fault == False
    assert len(msgs) == 1
    assert msgs[0].current_position.position_long == 0.123
    assert msgs[0].current_position.position_lat == -123.456


### Callback registration and parsing


def test_custom_message_callback(sensor_config_with_callback):
    """
    Test sensor servicing a message sent in "2 halves"
    """
    sensor, in_file_out, out_file_in, out_file_out, in_file_in = sensor_config_with_callback

    # Connect sensor and drone
    msg = I.ToSensor(gen_status=I.STATUS_OK)
    helpers.helper_send_message(msg, in_file_out)

    # Check that connection is properly initialized
    assert sensor.connect(timeout_s=1, tare_lat=0.1, tare_long=0.2) == True
    assert sensor.is_connection_active() == True

    # Send an obstacle message to sensor which should trigger the callback
    msg = I.ToSensor()
    waypoint = I.Waypoint(position_long=0.123, position_lat=-123.456)
    obstacle = I.Obstacle(radius_m=1.5)
    obstacle.location.CopyFrom(waypoint)
    msg.obstacle.CopyFrom(obstacle)
    helpers.helper_send_message(msg, in_file_out)

    # Service incoming messages, should trigger callback
    sensor.service_messages(timeout_ms=100)

    assert MSG_CALLBACK_DATA.obstacle.radius_m == 1.5
    assert MSG_CALLBACK_DATA.obstacle.location.position_long == 0.123
    assert MSG_CALLBACK_DATA.obstacle.location.position_lat == -123.456


### Message fault injection testing

def test_message_fault_incomplete(sensor_config):
    """
    Test sensor servicing a half message from the drone

    Should detect a message fault
    """
    sensor, in_file_out, out_file_in, out_file_out, in_file_in = sensor_config

    # Connect sensor and drone
    msg = I.ToSensor(gen_status=I.STATUS_OK)
    helpers.helper_send_message(msg, in_file_out)

    # Check that connection is properly initialized
    assert sensor.connect(timeout_s=1, tare_lat=0.1, tare_long=0.2) == True
    assert sensor.is_connection_active() == True

    # Send a complete message minus its last byte
    msg = I.ToSensor()
    waypoint = I.Waypoint(position_long=0.123, position_lat=-123.456)
    msg.current_position.CopyFrom(waypoint)
    msg_enc = helpers.helper_encode_message(msg)
    in_file_out.write(msg_enc[0 : len(msg_enc) - 1])
    in_file_out.flush()

    # Check that a message fault is detected
    msgs, fault = sensor.service_messages(timeout_ms=100)
    assert fault == True


def test_message_fault_corruption(sensor_config):
    """
    Test sensor servicing a corrupted message from the drone

    Should detect a message fault
    """
    sensor, in_file_out, out_file_in, out_file_out, in_file_in = sensor_config

    # Connect sensor and drone
    msg = I.ToSensor(gen_status=I.STATUS_OK)
    helpers.helper_send_message(msg, in_file_out)

    # Check that connection is properly initialized
    assert sensor.connect(timeout_s=1, tare_lat=0.1, tare_long=0.2) == True
    assert sensor.is_connection_active() == True

    # Send a complete message minus its last byte
    msg = I.ToSensor()
    waypoint = I.Waypoint(position_long=0.123, position_lat=-123.456)
    msg.current_position.CopyFrom(waypoint)
    msg_enc = helpers.helper_encode_message(msg)
    msg_enc = msg_enc[0:5] + b'\xff\xff\xff' +  msg_enc[5:]
    in_file_out.write(msg_enc)
    in_file_out.flush()

    # Check that a message fault is detected
    msgs, fault = sensor.service_messages(timeout_ms=100)
    assert fault == True

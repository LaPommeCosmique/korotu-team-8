"""
Smoke test drone_side.Drone() class
"""
import drone_side as D
import output.interface_pb2 as I
import transport_proto as T
import pytest
import helpers
import time


TEST_INPUT_FILE = "in.txt"
TEST_OUTPUT_FILE = "out.txt"

# Drone positions for tests
DRONE_POSITION_LAT_1 = 123.456
DRONE_POSITION_LONG_1 = 80.123

DRONE_POSITION_LAT_2 = -123.456
DRONE_POSITION_LONG_2 = 1.234

CURRENT_DRONE_POSITION = 1

def get_mock_drone_position():
    """
    Handler for getting a few unique drone positions for tests
    """
    if CURRENT_DRONE_POSITION == 1:
        return DRONE_POSITION_LONG_1, DRONE_POSITION_LAT_1
    elif CURRENT_DRONE_POSITION == 2:
        return DRONE_POSITION_LONG_2, DRONE_POSITION_LAT_2
    else:
        return 0., 0.


@pytest.fixture
def drone_config():
    """
    Setup and teardown basic drone class using files IO for testing
    """

    # Test setup

    # Opening test files with "w" mode should overwrite them
    out_file_out = open(TEST_OUTPUT_FILE, 'wb')
    in_file_out = open(TEST_INPUT_FILE, 'wb')
    in_file_in = open(TEST_INPUT_FILE, 'rb')
    out_file_in = open(TEST_OUTPUT_FILE, 'rb')

    drone = D.Drone(send_cb=out_file_out.write, rec_cb=in_file_in.read, get_pos_cb=get_mock_drone_position, all_msg_cbs=dict())
    yield drone, in_file_out, out_file_in, out_file_out, in_file_in

    # Test teardown

    in_file_in.close()
    out_file_out.close()
    in_file_in.close()
    out_file_in.close()


### Drone position tare and offset tests

def test_drone_set_tare(drone_config):
    """
    Run test on regular drone initialization with tare
    """

    drone, in_file_out, out_file_in, out_file_out, in_file_in = drone_config

    # Send connection OK keep alive
    msg = I.ToDrone(gen_status=I.STATUS_OK)
    helpers.helper_send_message(msg, in_file_out)

    # Send drone tare, longitude and latitude are +1 of the done
    msg = I.ToDrone()
    waypoint = I.Waypoint(position_long=(DRONE_POSITION_LONG_1+1), position_lat=(DRONE_POSITION_LAT_1+1))
    msg.tare_position.CopyFrom(waypoint)
    helpers.helper_send_message(msg, in_file_out)

    # Connect drone
    drone.connect(timeout_s=1)
    assert drone.is_connection_active() == True

    # Make sure tare offsets are exactly +1 of the current position
    assert drone.long_delta == 1.0
    assert drone.lat_delta == 1.0


def test_drone_set_and_reset_tare(drone_config):
    """
    Run test on regular drone initialization with tare
    """

    drone, in_file_out, out_file_in, out_file_out, in_file_in = drone_config

    # Send connection OK keep alive
    msg = I.ToDrone(gen_status=I.STATUS_OK)
    helpers.helper_send_message(msg, in_file_out)

    # Send drone tare, longitude and latitude are +1 of the done
    msg = I.ToDrone()
    waypoint = I.Waypoint(position_long=(DRONE_POSITION_LONG_1+1), position_lat=(DRONE_POSITION_LAT_1+1))
    msg.tare_position.CopyFrom(waypoint)
    helpers.helper_send_message(msg, in_file_out)

    # Connect drone
    drone.connect(timeout_s=1)
    assert drone.is_connection_active() == True

    # Reset the drone tare after connection initialized
    time.sleep(0.3)
    msg = I.ToDrone()
    waypoint = I.Waypoint(position_long=(DRONE_POSITION_LONG_1-2), position_lat=(DRONE_POSITION_LAT_1-2))
    msg.tare_position.CopyFrom(waypoint)
    helpers.helper_send_message(msg, in_file_out)

    # Let drone service messages
    drone.service_messages(timeout_ms=100)

    # Make sure tare offsets are the newest value offset from the current position
    assert drone.long_delta == -2.0
    assert drone.lat_delta == -2.0


### General drone -> sensor messages

def test_drone_send_update(drone_config):
    """
    Test drone sending general update
    """

    drone, in_file_out, out_file_in, out_file_out, in_file_in = drone_config

    # Send connection OK keep alive
    msg = I.ToDrone(gen_status=I.STATUS_OK)
    helpers.helper_send_message(msg, in_file_out)

    # Send drone tare, longitude and latitude are +1 of the done
    msg = I.ToDrone()
    waypoint = I.Waypoint(position_long=(DRONE_POSITION_LONG_1+1), position_lat=(DRONE_POSITION_LAT_1+1))
    msg.tare_position.CopyFrom(waypoint)
    helpers.helper_send_message(msg, in_file_out)

    # Connect drone
    drone.connect(timeout_s=1)
    assert drone.is_connection_active() == True
    msg = helpers.helper_read_and_decode_first_whole_message(out_file_out, out_file_in, I.ToSensor)

    # Send drone status message
    drone.send_drone_status(status=D.DRONE_STATUS_HOVERING, current_lat=(DRONE_POSITION_LAT_1+2), current_long=(DRONE_POSITION_LONG_1+2))

    # Ensure the drone status message is correct
    # Should be adjusted by the tare delta which is this case is +1 meter in the lat/long direction
    msg = helpers.helper_read_and_decode_first_whole_message(out_file_out, out_file_in, I.ToSensor)
    assert msg.drone_status == D.DRONE_STATUS_HOVERING
    assert msg.current_position.position_long == (DRONE_POSITION_LONG_1+3)
    assert msg.current_position.position_lat == (DRONE_POSITION_LAT_1+3)


def test_drone_send_obstacle(drone_config):
    """
    Test drone sending an obstacle detected message
    """
    drone, in_file_out, out_file_in, out_file_out, in_file_in = drone_config

    # Send connection OK keep alive
    msg = I.ToDrone(gen_status=I.STATUS_OK)
    helpers.helper_send_message(msg, in_file_out)

    # Send drone tare, longitude and latitude are +1 of the done
    msg = I.ToDrone()
    waypoint = I.Waypoint(position_long=(DRONE_POSITION_LONG_1+1), position_lat=(DRONE_POSITION_LAT_1+1))
    msg.tare_position.CopyFrom(waypoint)
    helpers.helper_send_message(msg, in_file_out)

    # Connect drone
    drone.connect(timeout_s=1)
    assert drone.is_connection_active() == True
    msg = helpers.helper_read_and_decode_first_whole_message(out_file_out, out_file_in, I.ToSensor)

    # Have drone send obstacle detected to sensor
    drone.send_obstacle(obstacle_pos_long=DRONE_POSITION_LONG_1-2, obstacle_pos_lat=DRONE_POSITION_LAT_1-2, obstacle_est_size=1.2)

    # Decode obstacle detected message, the obstacle position on the sensor side should be adjusted with the +1 tare delta in the lat/long directions
    msg = helpers.helper_read_and_decode_first_whole_message(out_file_out, out_file_in, I.ToSensor)
    assert msg.obstacle.location.position_long == (DRONE_POSITION_LONG_1-1)
    assert msg.obstacle.location.position_lat == (DRONE_POSITION_LAT_1-1)
    assert msg.obstacle.radius_m == 1.2

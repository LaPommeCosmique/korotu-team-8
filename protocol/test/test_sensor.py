"""
Smoke test sensor_side.Sensor() class

"""
import sensor_side as S
import output.interface_pb2 as I
import transport_proto as T
import pytest
import helpers


TEST_INPUT_FILE = "in.txt"
TEST_OUTPUT_FILE = "out.txt"


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


def test_sensor_update_waypoint(sensor_config):
    """
    Test sensor updating drone waypoint
    """
    sensor, in_file_out, out_file_in, out_file_out, in_file_in = sensor_config

    # Put response into buffer first
    msg = I.ToSensor(gen_status=I.STATUS_OK)
    helpers.helper_send_message(msg, in_file_out)

    # Check that connection is properly initialized
    assert sensor.connect(timeout_s=1, tare_lat=0.1, tare_long=0.2) == True
    assert sensor.is_connection_active() == True

    # First message should contain the keep alive and the second one should contain the tare position
    msg = helpers.helper_read_and_decode_first_whole_message(out_file_out, out_file_in, I.ToDrone)
    msg = helpers.helper_read_and_decode_first_whole_message(out_file_out, out_file_in, I.ToDrone)

    sensor.send_update_waypoint(pos_lat=-123.456, pos_long=456.123)
    msg = helpers.helper_read_and_decode_first_whole_message(out_file_out, out_file_in, I.ToDrone)
    assert msg.new_waypoint.position_long == 456.123
    assert msg.new_waypoint.position_lat == -123.456


def test_sensor_send_second_tare(sensor_config):
    """
    Test sensor updating drone tare
    """
    sensor, in_file_out, out_file_in, out_file_out, in_file_in = sensor_config

    # Put response into buffer first
    msg = I.ToSensor(gen_status=I.STATUS_OK)
    helpers.helper_send_message(msg, in_file_out)

    # Check that connection is properly initialized
    assert sensor.connect(timeout_s=1, tare_lat=0.1, tare_long=0.2) == True
    assert sensor.is_connection_active() == True

    # First message should contain the keep alive and the second one should contain the tare position
    msg = helpers.helper_read_and_decode_first_whole_message(out_file_out, out_file_in, I.ToDrone)
    msg = helpers.helper_read_and_decode_first_whole_message(out_file_out, out_file_in, I.ToDrone)

    sensor.send_tare_position(pos_lat=-123.456, pos_long=456.123)
    msg = helpers.helper_read_and_decode_first_whole_message(out_file_out, out_file_in, I.ToDrone)
    assert msg.tare_position.position_long == 456.123
    assert msg.tare_position.position_lat == -123.456


# Drone - Sensor Communication API
V1.0


## Application Layer

In general, the sensor and drone will run asynchronously. As such, both devices must service any incoming messages at a minimum of
a 500 ms cycle time.

## Connection Initialization (Sender: Both, Receiver: Both)
Both drone and sensor can initialize a connection by begining to send periodic keep-alive messages over the link.

## Tare Position (Sender: Sensor, Receiver: Drone)
After connection has been established, the sensor shall send a "tare position", with the sensor's current estimated latitude and longitude. After this, all positions on the drone side shall be tared to this relative position.

### Keep Alive (Sender: Both, Receiver: Both)
Both drone and sensor shall broadcast a "keep-alive" message at least once per second containing STATUS_OK. If
either the drone or sensor fails to receive this keep-alive, the connection will be considered terminated.

### Terminating Connection (Sender: Both, Receiver: Both)
If either the drone or sensor misses 2 consecutive keep-alives OR if a message containing STATUS_FAULT is received by either party,
then the connection will be considered terminated and all parameters void. The device which first receives the STATUS_FAULT message or does not receive the consecutive keep alive's must also broadcast STATUS_FAULT on the bus.

### Drone Status (Sender: Drone, Receiver: Sensor)
Drone shall send its current position, speed and next waypoint (if underway) along with its general status (flying or grounded)
at a 1s periodicity.

### Update Next Waypoint (Sender: Sensor, Receiver: Drone)
Sensor can asynchronously, at any time, set the waypoint for the drone to attempt to navigate to immediately.

### Obstacle Detection (Sender: Drone, Receiver: Sensor)
If an obstacle is detected by the drone in between its current position and the waypoint it is currently attempting to move toward,
the drone will hover in place, clear the next waypoint and finally send an obstacle detection message to the sensor
containing the obstacle coordinates and its estimated radius in meters. The sensor can resume the drone movement path by
sending a "Update Next Waypoint" command.


## Transport Layer

Application layer messages are encoded and serialized with the protobuf protocol defined in `interface.proto`.

Messages are sent in the following format:

\<msg length\>\<serialized_protobuf\>

## Link Layer

Drone and Sensor communicate over 115200 baud UART.

## Physical Layer

Drone and Sensor connected by 3 wire UART phy (Ground, TXD, RXD). USB to UART connector can be used on either side if needed. The cable should be maximum 1 meter long to reduce signal noise.
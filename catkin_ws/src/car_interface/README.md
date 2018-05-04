# Car Interface #
This is a package of ROS nodes and messages which provides the core functionality for the EB challenge car.

## Configuration parmeters

The nodes need to know the tty for the serial port, where the Teensy is conected.
Also needed is the communcations speed. The default is 115200b/s.

## ROS Topics
This code define 4 ROS topics.

### Topic "teensy_pong"
This topic defines two timestamps. It is sent to Teensy with RPi time set in
`timestamp1`, and `timestamp2` set to zero ans a PING command.

The Teensy will return the message as a PONG reply and set timestamp2 to it's local time, when responding.
When RPi received the response the difference between timestamp1 and time of reception,
indicates the round-trip delay.

Half of this can be used as an estimate of the latency from Teensy to RPi.
The timestamp2 value can be used to establish a mapping between Teensy time (`millis()`) and the time on RPi (ROS) side.

```
# ping sender fill in this (pong sender copies)
uint32 timestamp1
# ping sets to 0 (pong sender fill this)
uint32 timestamp2
```

### Topic "teensy_wheel"
Data for wheel motion is sent for both `left` and `right` wheel.

```
# Two records, one for left and one for right wheel
OneWheel left
OneWheel right
```
Each wheel has the following data defined.

```
# Pulses per second (lowest possible speed is 20 seconds for one wheel revolution)
uint16 speed
# Enumeration rotDirection is used
uint8 direction
# Timestamp for measurement
uint32 when
# Odometer when direction changed
uint32 dist
# Absolute distance travelled
uint32 dist_abs
```

### Topic "teensy_distance"
This topic reports the ultrasound distance sensor measurements.
In the current setup there are 6 sensors (`sensor`) supported, numbered from 0 to 5.

The distance measurement (`distance`) is reported in microseconds for echo to return.
This should be converted to m/s using the speed of sound and possibly the temperature for fine adjustment.

The timestamp (`when`) is the local Teensy time. See `teensy_pong` topic for how to convert this to RPi time.
This may be important in order to get correct position of obstacle, when car is moving and
there is some latency in processing the distance measurements.

```
# 0 .. MAX_NO_OF_SONAR-1
uint8 sensor
# Measurements in microseconds (time to hear echo @ speed of sound)
uint16 distance
# Time when data was measured
uint32 when
```

### Topic "teensy_error"
The Teensy keeps track of a number of different error counters. Each counter is reported as a separate message.
The `count` indicates how many times this situation has occurred.
The `name` is the name of the counter. The Teensy code identifies a counter with this string (there is a easily managed list of counter names and descriptions in the Teensy source code).

```
# Number of occurrence
uint32 count
# Name of counter
string name
```


## Control
Currently the node accepts simple keypresses (from the relevant terminal) as a way to send controls to the Teensy software.

# Installation
Move to `./catkin_ws/src` and extract the repository there

This should end up with a path like `~/catkin_ws/src/teensy_sensor_hub`

From directory `./catkin_ws` run `catkin_make`, followed by `catkin_make install`


#!/usr/bin/env python
"""This module implements the ROS interface to Teensy sensorhub"""
from __future__ import print_function
import select
import sys
import termios
import atexit
import serial
import sensorhub
import rospy
from car_interface.msg import Wheels
from car_interface.msg import Distance
from car_interface.msg import Error
from car_interface.msg import PingPong

from car_control import CarControl
# MOTOR_MAX_THROTTLE_PERCENTAGE = 50
# MOTOR_MAX_REVERSE_PERCENTAGE = -50
# MOTOR_STEP_SIZE_PERCENT = 5
# MOTOR_BREAKING_FORCE = 5
# MOTOR_BREAKING_DELAY = 0.3

cc = CarControl()

# pylint: disable=C0103
# pylint: disable=W0603

# ROS topic names
PONG_TOPIC_NAME = "teensy_pong"
WHEEL_TOPIC_NAME = 'teensy_wheel'
DISTANCE_TOPIC_NAME = 'teensy_distance'
ERROR_TOPIC_NAME = "teensy_error"

# ROS Publishers
_publishWheels = None
_publishDistance = None
_publishPingPong = None
_publishError = None

def setupPublishers():
    """Initialize interface objects for publishing teensy topics"""
    global _publishWheels
    global _publishDistance
    global _publishPingPong
    global _publishError
    _publishWheels = rospy.Publisher(WHEEL_TOPIC_NAME, Wheels, queue_size=1)
    _publishDistance = rospy.Publisher(DISTANCE_TOPIC_NAME, Distance, queue_size=1)
    _publishPingPong = rospy.Publisher(PONG_TOPIC_NAME, PingPong, queue_size=1)
    _publishError = rospy.Publisher(ERROR_TOPIC_NAME, Error, queue_size=1)
    return

def handlePingPong(ping):
    """Handle ping or pong message. Used to determine delay in comms with Teensy"""
    global _publishPingPong
    PingpongMsg = PingPong()
    PingpongMsg.timestamp1 = ping.timestamp1
    PingpongMsg.timestamp2 = ping.timestamp2
    rospy.loginfo(PingpongMsg)
    _publishPingPong.publish(PingpongMsg)
    return

def handleWheels(left, right):
    """Handle wheel odometer messages. Data for left and right wheel are separate"""
    global _publishWheels
    wheelsMsg = Wheels()

    wheelsMsg.left.when = left.when
    wheelsMsg.right.when = right.when
    wheelsMsg.left.speed = left.speed #TODO: m/s
    wheelsMsg.right.speed = right.speed
    wheelsMsg.left.direction = left.direction
    wheelsMsg.right.direction = right.direction
    wheelsMsg.left.dist = left.turn
    wheelsMsg.right.dist = right.turn
    wheelsMsg.left.dist_abs = right.dist #TODO: convert to meters
    wheelsMsg.right.dist_abs = right.dist
    #rospy.loginfo(wheelsMsg)
    _publishWheels.publish(wheelsMsg)
    return

def handleDist(dist):
    """Handle distance measurements from ultrasound sensors"""
    global _publishDistance
    distanceMsg = Distance()
    distanceMsg.sensor = dist.sensor
    distanceMsg.distance = dist.distance # convert from us to m
    distanceMsg.when = dist.when  # adjust time to RPi time
    #rospy.loginfo(distanceMsg)
    _publishDistance.publish(distanceMsg)
    return

def handleError(error):
    """Handle error counter updates from Teensy"""
    global _publishError
    errorMsg = Error()
    errorMsg.count = error.count
    errorMsg.name = error.name
    rospy.loginfo(errorMsg)
    _publishError.publish(errorMsg)
    return

_old_settings = None

def init_anykey():
    """Setup stdin to handle single keypresses. This is primitive way to have some control"""
    global _old_settings
    _old_settings = termios.tcgetattr(sys.stdin)
    new_settings = termios.tcgetattr(sys.stdin)
    new_settings[3] = new_settings[3] & ~(termios.ECHO | termios.ICANON) # lflags
    new_settings[6][termios.VMIN] = 0  # cc
    new_settings[6][termios.VTIME] = 0 # cc
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, new_settings)
    atexit.register(term_anykey)
    return

def term_anykey():
    """Restore terminal input setting. Called from atexit handler"""
    global _old_settings
    if _old_settings:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, _old_settings)
    return

# User input handling
def handleKeyPress(sensor, key):
    """Handle single keypresses as commands to Teensy"""
    key = key.upper()
    status = True
    if key == 'P':
        sensor.sendPing()
    elif key == 'N':
        sensor.sendSonarStop()
    elif key == 'S':
        sensor.sendSonarStart()
    elif key == 'R':
        sensor.sendWheelReset()
    elif key == 'C':
        sensor.sendGetCounters()
    elif key == 'X':
        sensor.sendSonarSequence([0, 1, 5])
    elif key == 'Y':
        sensor.sendSonarSequence([0, 1, 2, 3, 4, 5])
    elif key == 'Q':
        status = False
    return status
        
def talker():
    """Main function of Teensy ROS interface"""
    # TODO: this is a hack, using the old code to read keyboad directly.
    # Replace with ROS oriented implementation
    init_anykey()

    rospy.init_node('sensor_interface_node', anonymous=True)
    setupPublishers()
    #r = rospy.Rate(10) #10hz

    sh = sensorhub.Sensorhub()
    sh.setOutputHandlers(handlePingPong, handlePingPong, handleWheels, handleDist, handleError)

    # TODO: Get serial port setting to teensy connection - check ROS config parameter handling
    #if len(sys.argv) > 1:
    #    port = sys.argv[1]
    #else:
    #    port = '/dev/ttyAMA0'
    port = '/dev/ttyAMA0'
    baud = 115200
    ser = serial.Serial(port, baud, timeout=0)

    while not rospy.is_shutdown():
        if sh.txDataAvailable():
            # If we have tx data when also wait to serial tx readiness
            txList = [ser]
        else:
            txList = []
        r, w, e = select.select([sys.stdin, ser], txList, [], 0.010)

        # data coming from keyboard?
        if sys.stdin in r:
            if not handleKeyPress(sh, sys.stdin.read(1)):
                break  # bail out

        # data coming from serial?
        if ser in r:
            data_str = ser.read(ser.inWaiting())
            for b in data_str:
                sh.rxRawByte(b)

        # Any data to transmit over serial?
        if ser in w:
            flagAndByte = sh.txGetByte()
            if flagAndByte[0]:
                ser.write(chr(flagAndByte[1]))

    return

def myhook():
    print("SHUTTING DOWN sensor_interface!")
  
if __name__ == '__main__':
	try:
		talker()
		rospy.on_shutdown(myhook)
	except rospy.ROSInterruptException:
		pass

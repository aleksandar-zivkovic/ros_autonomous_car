#!/usr/bin/env python
"""This module contains a simple example of using the Teensy sensor hub with a terminal."""
from __future__ import print_function
import select
import sys
import termios
import atexit
import serial
import sensorhub

# pylint: disable=C0103
# pylint: disable=W0603

def locate(user_string, cursor_x=0, cursor_y=0):
    """Position terminal cursor at indicated coordinates"""
    # Don't allow any user errors. Python's own error detection will check for
    # syntax and concatination, etc, etc, errors.
    _x = int(cursor_x)
    _y = int(cursor_y)
    if _x >= 255:
        _x = 255
    if _y >= 255:
        _y = 255
    if _x <= 0:
        _x = 0
    if _y <= 0:
        _y = 0
    horiz = str(_x)
    vert = str(_y)
    # Plot the user_string at the starting at position HORIZ, VERT...
    print("\033["+vert+";"+horiz+"f"+user_string)
    return

def prettyPrintDist(dist):
    """Print ultrasound distance sensor at sensors screen position"""
    sensor = dist.sensor
    locate(' {:10} mm'.format(dist.distance), 15*sensor, 1)
    locate(' {:10} ms'.format(dist.when), 1, 8)
    locate("", 0, 9)
    return

def prettyPrintWheels(left, right):
    """Print wheel odometer data"""
    locate(' {:10} pulses  {:10} pulses'.format(left.dist, right.dist), 1, 3)
    locate(' {:10} p/s     {:10} p/s'.format(left.speed, right.speed), 1, 4)
    locate(' {:10} dir     {:10} dir'.format(left.direction, right.direction), 1, 5)
    locate(' {:10} turn    {:10} turn'.format(left.turn, right.turn), 1, 6)
    locate(' {:10} ms'.format(left.when), 1, 8)
    locate("", 0, 9)
    return

def prettyPrintPong(resp):
    """Print ping time data"""
    now = sensorhub.getMilliSeconds()
    locate('{:10}ms  {:10}ms'.format(now -resp.timestamp1, resp.timestamp2), 25, 8)
    return

errorPos = {}
errLin = 12

def errorLine(name):
    """Print error counter at it's own line"""
    global errorPos, errLin
    if not name in errorPos:
        errorPos[name] = errLin
        errLin += 1
    return errorPos[name]

def prettyPrintError(err):
    """Handle error counter"""
    locate("{:20}: {:10}".format(err.name, err.count), 1, errorLine(err.name))
    return


def clearScreen():
    """Send clear screen escape code"""
    print("\033[2J")
    return


# User input handling
def handleKeyPress(sensor, key):
    """Decode keypresses and act"""
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
    elif key == '\x0C':
        clearScreen()
    elif key == 'X':
        sensor.sendSonarSequence([0, 1, 5])
    elif key == 'Y':
        sensor.sendSonarSequence([0, 1, 2, 3, 4, 5])
    elif key == 'Q':
        status = False
    return status


old_settings = None

def init_anykey():
    """ Setup terminal to handle single keystokes directly"""
    global old_settings
    old_settings = termios.tcgetattr(sys.stdin)
    new_settings = termios.tcgetattr(sys.stdin)
    new_settings[3] = new_settings[3] & ~(termios.ECHO | termios.ICANON) # lflags
    new_settings[6][termios.VMIN] = 0  # cc
    new_settings[6][termios.VTIME] = 0 # cc
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, new_settings)
    atexit.register(term_anykey)
    return

def term_anykey():
    """Reset terminal to "normal" settings when exiting"""
    global old_settings
    if old_settings:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return

B = "\033[1;31m"
N = "\033[0m"

def main():
    """Main function"""
    sh = sensorhub.Sensorhub()
    sh.setOutputHandlers(None, \
                         prettyPrintPong, \
                         prettyPrintWheels, \
                         prettyPrintDist, \
                         prettyPrintError)
    clearScreen()

    locate(B+"P"+N+"ing So"+\
           B+"n"+N+"arstop "+\
           B+"S"+N+"onarstart Wheel"+\
           B+"R"+N+"eset "+\
           B+"C"+N+"ounters "+\
           B+"X"+N+"=seq1 "+\
           B+"Y"+N+"=seq2", 1, 10)
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = '/dev/ttyAMA0'
    baud = 115200
    ser = serial.Serial(port, baud, timeout=0)

    init_anykey()
    #xpos=1
    while True:
        if sh.txDataAvailable():
            txList = [ser]
        else:
            txList = []
        r, w, e = select.select([sys.stdin, ser], txList, [])

        # data coming from keyboard?
        if sys.stdin in r:
            #locate("\033[K",1,30)
            #xpos=1
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
                #locate(format(flagAndByte[1],"02x"),xpos,30)
                #xpos += 2

    return

if __name__ == '__main__':
    main()

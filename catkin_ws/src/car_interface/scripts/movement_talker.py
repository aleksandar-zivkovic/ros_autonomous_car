#! /usr/bin/env python
from __future__ import print_function

import select
import sys, tty
import termios
import atexit
import serial
import sensorhub
import rospy

# Brings in the messages
import car_interface.msg
from car_interface.msg import Movement

def getin():
	fd = sys.stdin.fileno()
	old_settings = termios.tcgetattr(fd)
	try:
		tty.setraw(sys.stdin.fileno())
		ch = sys.stdin.read(1)
	finally:
		termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
	return ch

def main_loop():
	rospy.init_node('movement_talker_node')
	pub = rospy.Publisher('movement_talker', Movement, queue_size=1)
	#    print "Enter your command (i=forward, o=steer right, u=steer left, k=reverse, r=reset steering, 0=Stop!):"
	#    print "Enter 'q' to quit!"
	print(',---------------------------------------------')
	print('| Enter command:                              |')
	print('| w = throttle forward / s = throttle reverse |')
	print('| a = steer left / d = steer right            |')
	print('| b = break                                   |')
	print('| r = center steering                 	     |')
	print('| any other key = kill switch, STOP!          |')
	print('| q = STOP and quit script                    |')
	print(' ---------------------------------------------')

	while True: 
		ch = getin()
		movement_msg = Movement()
		if ch == 'w' or ch == 'W':
			#move forward
			movement_msg.command = 'forward'
			pub.publish(movement_msg)
		elif ch == 's' or ch == 'S':
			#move backward
			movement_msg.command = 'reverse'
			pub.publish(movement_msg)
		elif ch == 'd' or ch == 'D':
			#turn_right
			movement_msg.command = 'right'
			pub.publish(movement_msg)
		elif ch == 'a' or ch == 'A':
			#turn_left
			movement_msg.command = 'left'
			pub.publish(movement_msg)
		elif ch == 'r' or ch == 'R':
			#reset_steering
			movement_msg.command = 'reset'
			pub.publish(movement_msg)
		elif ch == 'b' or ch == 'B': 
			#active_braking
			movement_msg.command = 'brake'
			pub.publish(movement_msg)
		elif ch == 'k' or ch == 'K': 
			#k turn right
			movement_msg.command = 'kright'
			pub.publish(movement_msg)
		elif ch == 'l' or ch == 'L': 
			#k turn left
			movement_msg.command = 'kleft'
			pub.publish(movement_msg)
		elif ch == 'o' or ch == 'O': 
			# turn to x degree
			movement_msg.command = 'turn'
			pub.publish(movement_msg)
		elif ch == 'm' or ch == 'M': 
			# move x meters
			movement_msg.command = 'move'
			pub.publish(movement_msg)
		elif ch == 'q' or ch == 'Q':
			movement_msg.command = 'stop'
			pub.publish(movement_msg)
			sys.exit(0)
		else:
			raise KeyboardInterrupt
			#stop_and_reset
			movement_msg.command = 'stop'
			pub.publish(movement_msg)

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion.", file=sys.stderr)

#!/usr/bin/env python
import rospy
import time
from car_control import CarControl
from car_interface.msg import Movement, Distance, Wheels, ImuMessage
from sensor_msgs.msg import Imu, Temperature, MagneticField


from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# ROS topic names
MOVEMENT_TOPIC_NAME = 'movement_talker'
DISTANCE_TOPIC_NAME = 'teensy_distance'
PONG_TOPIC_NAME = "teensy_pong"
WHEEL_TOPIC_NAME = 'teensy_wheel'
ERROR_TOPIC_NAME = "teensy_error"
IMU_TALKER_TOPIC_NAME = 'imu_talker'

OBSTACLE_DETECTION_DISTANCE = 200

# Global boolean variables used as flags for movement block 
sensor_block_0 = False # front right
sensor_block_1 = False # front middle
sensor_block_2 = False # front left
sensor_block_3 = False # back right
sensor_block_4 = False # back middle 
sensor_block_5 = False # back left

# Global float variables for orientation of the car in euler orientation
heading = 0.0
roll = 0.0
pitch = 0.0

# Global variables for wheel odometry count
wheel_left = 0
wheel_right = 0

# Calculated during wheel odometry calibration, one meter is ~64 counts (TODO: Check)
COUNTS_PER_ONE_METER = 64

cc = CarControl()

def move_x_meters(x_meters):
	target_wheel_left = wheel_left.dist + x_meters * COUNTS_PER_ONE_METER
	target_wheel_right = wheel_right.dist + x_meters * COUNTS_PER_ONE_METER
	
	cc.more_gas()
	cc.more_gas()
	while wheel_left.dist < target_wheel_left and wheel_right.dist < target_wheel_right:
		if (sensor_block_0 or sensor_block_1 or sensor_block_2 or sensor_block_3 or sensor_block_4 or sensor_block_5):
				cc.stop_and_reset()
				break
	cc.stop_and_reset()	

def turn_to_target_orientation(int_target_heading):
	# figure out which direction to turn 
	int_heading = int(heading)
	direction = 0
	
	if (int_heading != int_target_heading):
		diff = int_target_heading - int_heading
		if diff < 0:
			diff = diff + 360
		if diff > 180:
			direction = -1; # left turn
		else:
			direction = 1; # right turn
	else:
		rospy.loginfo("Car is already heading %d" % (int_target_heading))
		return

	cc.more_gas()
	cc.more_gas()
	# left
	if direction == -1:
		while int_heading != int_target_heading:
			int_heading = int(heading)
			rospy.loginfo("Int Goal: %d" % (int_target_heading))
			rospy.loginfo("Int heading: %d" % (int_heading))
			if (sensor_block_0 or sensor_block_1 or sensor_block_2 or sensor_block_3 or sensor_block_4 or sensor_block_5):
				cc.stop_and_reset() #remove
				break
			else:
				cc.steer_left()
	else:
		while int_heading != int_target_heading:
			int_heading = int(heading)
			rospy.loginfo("Int Goal: %d" % (int_target_heading))
			rospy.loginfo("Int heading: %d" % (int_heading))
			if (sensor_block_0 or sensor_block_1 or sensor_block_2 or sensor_block_3 or sensor_block_4 or sensor_block_5):
				cc.stop_and_reset() #remove
				break
			else:
				cc.steer_right()

	cc.stop_and_reset()
	return
	
def k_turn_left():
	int_heading = int(heading)
	int_target_heading = abs(int_heading - 90)
	
	cc.reverse()
	cc.reverse()
	while int_heading != int_target_heading:
		int_heading = int(heading)
		rospy.loginfo("Int heading: %d" % (int_heading))
		if (sensor_block_0 or sensor_block_1 or sensor_block_2 or sensor_block_3 or sensor_block_4 or sensor_block_5):
			cc.stop_and_reset() #remove
			break
		else:
			cc.steer_right()
			
	cc.stop_and_reset()
	
	int_target_heading = abs(int_heading - 90)
	cc.more_gas()
	cc.more_gas()
	while int_heading != int_target_heading:
		int_heading = int(heading)
		if (sensor_block_0 or sensor_block_1 or sensor_block_2 or sensor_block_3 or sensor_block_4 or sensor_block_5):
			cc.stop_and_reset() #remove
			break
		else:
			cc.steer_left()
	cc.stop_and_reset()
    
def movement_callback(movement_msg):
	
	rospy.loginfo("Movement command received: %s" % (movement_msg.command))
	if movement_msg.command == 'forward':
		if sensor_block_0 == False and sensor_block_1 == False and sensor_block_2 == False:
			cc.more_gas()
	elif movement_msg.command == 'reverse':
		if sensor_block_3 == False and sensor_block_4 == False and sensor_block_5 == False:
			cc.reverse()
	elif movement_msg.command == 'left':
		cc.steer_left()
	elif movement_msg.command == 'right':
		cc.steer_right()
	elif movement_msg.command == 'brake':
		cc.active_braking()
	elif movement_msg.command == 'turn':
		turn_to_target_orientation(0)
	elif movement_msg.command == 'move':
		move_x_meters(1)
	elif movement_msg.command == 'kleft':
		k_turn_left()
	#elif movement_msg.command == 'kright':
		#k_turn_right()
	elif movement_msg.command == 'reset':
		cc.reset_steering()
	else:
		cc.stop_and_reset()

# Ultrasound sensor test        
def ultrasound_sensors_test_callback(distance_msg):
	if distance_msg.distance < 100:
		rospy.loginfo("Sensor number %d: Distance: %s" % (distance_msg.sensor, distance_msg.distance))
	
def distance_callback(distance_msg):

	global sensor_block_0 # front right
	global sensor_block_1 # front middle
	global sensor_block_2 # front left
	global sensor_block_3 # back right
	global sensor_block_4 # back middle 
	global sensor_block_5 # back left
	
	if distance_msg.sensor == 0:
		#rospy.loginfo("Sensor %d block status: %s." % (distance_msg.sensor, sensor_block_0))
		if distance_msg.distance < OBSTACLE_DETECTION_DISTANCE:
			if (sensor_block_0 == False):
				cc.active_braking()
				rospy.loginfo("Obsticle detected by sensor %d." % (distance_msg.sensor))
				sensor_block_0 = True
				rospy.loginfo("Forward movement blocked by sensor: %d." % (distance_msg.sensor))
		else:
			sensor_block_0 = False
	elif distance_msg.sensor == 1:
		#rospy.loginfo("Sensor %d block status: %s." % (distance_msg.sensor, sensor_block_1))
		if distance_msg.distance < OBSTACLE_DETECTION_DISTANCE:
			if (sensor_block_1 == False):
				cc.active_braking()
				rospy.loginfo("Obsticle detected by sensor %d." % (distance_msg.sensor))
				sensor_block_1 = True
				rospy.loginfo("Forward movement blocked by sensor: %d." % (distance_msg.sensor))
		else:
			sensor_block_1 = False
	elif distance_msg.sensor == 2:
		#rospy.loginfo("Sensor %d block status: %s." % (distance_msg.sensor, sensor_block_2))
		if distance_msg.distance < OBSTACLE_DETECTION_DISTANCE:
			if (sensor_block_2 == False):
				cc.active_braking()
				rospy.loginfo("Obsticle detected by sensor %d." % (distance_msg.sensor))
				sensor_block_2 = True
				rospy.loginfo("Forward movement blocked by sensor: %d." % (distance_msg.sensor))
		else:
			sensor_block_2 = False
	#===============================================================================
	elif distance_msg.sensor == 3:
		#rospy.loginfo("Sensor %d block status: %s." % (distance_msg.sensor, sensor_block_3))
		if distance_msg.distance < OBSTACLE_DETECTION_DISTANCE:
			if (sensor_block_3 == False):
				cc.active_braking()
				rospy.loginfo("Obsticle detected by sensor %d." % (distance_msg.sensor))
				sensor_block_3 = True
				rospy.loginfo("Reverse movement blocked by sensor: %d." % (distance_msg.sensor))
		else:
			sensor_block_3 = False
	elif distance_msg.sensor == 4:
		#rospy.loginfo("Sensor %d block status: %s." % (distance_msg.sensor, sensor_block_4))
		if distance_msg.distance < OBSTACLE_DETECTION_DISTANCE:
			if (sensor_block_4 == False):
				cc.active_braking()
				rospy.loginfo("Obsticle detected by sensor %d." % (distance_msg.sensor))
				sensor_block_4 = True
				rospy.loginfo("Reverse movement blocked by sensor: %d." % (distance_msg.sensor))
		else:
			sensor_block_4 = False
	elif distance_msg.sensor == 5:
		#rospy.loginfo("Sensor %d block status: %s." % (distance_msg.sensor, sensor_block_5))
		if distance_msg.distance < OBSTACLE_DETECTION_DISTANCE:
			if (sensor_block_5 == False):
				cc.active_braking()
				rospy.loginfo("Obsticle detected by sensor %d." % (distance_msg.sensor))
				sensor_block_5 = True
				rospy.loginfo("Reverse movement blocked by sensor: %d." % (distance_msg.sensor))
		else:
			sensor_block_5 = False
	
def imu_message_callback(data):
	global heading 
	global roll 
	global pitch 
	
	heading = data.heading
	roll = data.roll
	pitch = data.pitch
	
	print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}'.format(heading, roll, pitch))
	
	
def wheels_callback(data):
	# Handle WHEELS topic 
	
	global wheel_left
	global wheel_right
	
	wheel_left = data.left
	wheel_right = data.right
	
	#rospy.loginfo("Left wheel: speed=%d, direction=%d, timestamp=%d, distance=%d, abs_distance=%d" % (wheel_left.speed, wheel_left.direction, wheel_left.when, wheel_left.dist, wheel_left.dist_abs))
	#rospy.loginfo("Right wheel: speed=%d, direction=%d, timestamp=%d, distance=%d, abs_distance=%d" % (wheel_right.speed, wheel_right.direction, wheel_right.when, wheel_right.dist, wheel_right.dist_abs))
          
def movement_listener():
	
	rospy.init_node('movement_listener_node', anonymous=True)
	
	rospy.Subscriber(MOVEMENT_TOPIC_NAME, Movement, movement_callback)
	rospy.Subscriber(DISTANCE_TOPIC_NAME, Distance, distance_callback)
	rospy.Subscriber(WHEEL_TOPIC_NAME, Wheels, wheels_callback)
	rospy.Subscriber(IMU_TALKER_TOPIC_NAME, ImuMessage, imu_message_callback)

    # keeps python from exiting until this node is stopped
	rospy.spin()
	
def shutdown_hook():
	cc.stop_and_reset()
	print "Shutting down!"

if __name__ == '__main__':
	try:
		movement_listener()
		rospy.on_shutdown(shutdown_hook)
	except rospy.ROSInterruptException:
		pass


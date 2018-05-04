#! /usr/bin/env python

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

import rospy
# Brings in the messages
import car_interface.msg
from car_interface.msg import Movement

def main_loop():
	rospy.init_node('camera_talker_node')
	pub = rospy.Publisher('movement_talker', Movement)
	
	# initialize the camera and grab a reference to the raw camera capture
	camera = PiCamera()
	camera.resolution = (640, 480)
	camera.framerate = 64
	camera.hflip = True
	camera.vflip = True
	rawCapture = PiRGBArray(camera, size=(640, 480))

	# lower and upper bounds of each color
	blue = [([86,31,4],[220,88,50])]
	red = [([17,15,100],[50,56,200])]
	yellow = [([29,130,208],[62,174,250])]
	green =[([35,109,21],[83,250,106])]
	 
	# allow the camera to warmup
	time.sleep(0.1)
	 
	try:
		# capture frames from the camera
		for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
			# grab the raw NumPy array representing the image, then initialize the timestamp and occupied/unoccupied text
			image = frame.array
			# middle of the screen as AIM for image detection only
			cube = image[250:300,200:250]
			#blue
			for (lower, upper) in blue:
				lower = np.array(lower,dtype="uint8")
				upper = np.array(upper,dtype="uint8")
			mask = cv2.inRange(cube,lower,upper)
			b = sum(sum(mask))
			print ("blue")
			print (b)
			#red
			for(lower,upper) in red:
				lower = np.array(lower,dtype="uint8")
				upper = np.array(upper,dtype="uint8")
			mask = cv2.inRange(cube,lower,upper)
			r = sum(sum(mask))
			print ("red")
			print (r)
			#green
			for(lower,upper) in green:
				lower = np.array(lower,dtype="uint8")
				upper = np.array(upper,dtype="uint8")
			mask = cv2.inRange(cube,lower,upper)
			g = sum(sum(mask))
			print ("green")
			print (g) 
			#yellow	
			for(lower,upper) in yellow:
				lower = np.array(lower,dtype="uint8")
				upper = np.array(upper,dtype="uint8")
			mask = cv2.inRange(cube,lower,upper)
			y = sum(sum(mask))
			print ("yellow")
			print (y)
			if b < 1000 and r < 1000 and y < 1000 and g < 1000:
				print ("no block")
			else:
				movement_msg = Movement() 
				if b > r and b > g and b >y:
					print ("block blue")
					#move reverse
					movement_msg.command = 'reverse'
					pub.publish(movement_msg)
				elif r > g and r > y:
					print ("block red")
					#brake
					movement_msg.command = 'brake'
					pub.publish(movement_msg)
				elif y > g:
					print ("block yellow")
				else:
					print ("block green")
					#move forward
					movement_msg.command = 'forward'
					pub.publish(movement_msg)

			cv2.imshow("cube",cube)
			print (cube [30][30])
			cv2.rectangle(image,(200,250),(250,300),(0,0,255),2)
			cv2.imshow("original", image)
			
			key = cv2.waitKey(1) & 0xFF

			# clear the stream in preparation for the next frame
			rawCapture.truncate(0)

			# if the `q` key was pressed, break from the loop
			if key == ord("q"):
				break

	except KeyboardInterrupt:
		pass
	
if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion.")

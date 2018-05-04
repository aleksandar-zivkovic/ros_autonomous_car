#!/usr/bin/env python
from __future__ import print_function
#import roslib; roslib.load_manifest('YOUR_PACKAGE_NAME_HERE')
import rospy
#import tf.transformations
from geometry_msgs.msg import Twist
from car_control import CarControl
from car_interface.msg import Wheels

car_controller = None
global_stop = False

def cmd_vel_callback(msg):
    if global_stop:
        return
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
 
    # Do velocity processing here:
    # Use the kinematics of your robot to map linear and angular velocities into motor commands
 
    #v_l = ...
    #v_r = ...
 
    # Then set your wheel speeds (using wheel_left and wheel_right as examples)
    wheel_left.set_speed(v_l)
    wheel_right.set_speed(v_r)
    return

def teensy_wheel_callback(msg):
    global car_controller
    if global_stop:
        return
    #rospy.loginfo(msg)
    rospy.loginfo("Received a /teensy_wheel message! [%u %u] "%(msg.left.direction, msg.left.dist))
    if msg.left.direction > 0:
        car_controller.steer_left()
        car_controller.more_gas()
    else:
        car_controller.steer_right()
        car_controller.reverse()
    return

def shutdown():
    global car_controller
    global global_stop
    global_stop = True
    print("shutdown called")
    car_controller.motor_stop_and_center()
    return

def listener():
    global car_controller
    car_controller = CarControl()
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    rospy.Subscriber("/teensy_wheel", Wheels, teensy_wheel_callback)
    rospy.on_shutdown(shutdown)
    rospy.spin()
    return
 
if __name__ == '__main__':
    listener()

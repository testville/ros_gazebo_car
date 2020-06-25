#!/usr/bin/env python
import math
import numpy
import threading

from math import pi

import rospy
import tf

from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float64
from controller_manager_msgs.srv import ListControllers

def pubSteer(data):
    leftSteerPub = rospy.Publisher('front_left_steering_knuckle_controller/command', Float64, queue_size=10)
    rightSteerPub = rospy.Publisher('front_right_steering_knuckle_controller/command', Float64, queue_size=10)
    frontLeftVelPub = rospy.Publisher('front_left_wheel_vel_controller/command', Float64, queue_size=10)
    frontRightVelPub = rospy.Publisher('front_right_wheel_vel_controller/command', Float64, queue_size=10)
    inputVelocity = (data.speed / (2 * pi * 0.2)) * 2 * pi # linear vel to angular vel
    inputTurningAngle = data.steering_angle
    lengthBetweenWheelBase = 0.4 * 3.4 * 0.7
    width  = 0.8
    rospy.loginfo(inputTurningAngle)
    sign  = 1
    if inputTurningAngle > 0 : 
        sign = 1
    else:
        sign = -1
    t = math.tan(inputTurningAngle)
    thetaZero = 0
    thetaOne = 0
    if abs(t) > 0.0000001:
	radius = lengthBetweenWheelBase / t
    	thetaZero = math.atan(lengthBetweenWheelBase/(radius - width / 2.0)) 
    	thetaOne = math.atan(lengthBetweenWheelBase/(radius + width / 2.0))
    rospy.loginfo(thetaZero)
    rospy.loginfo(thetaOne)
    leftSteerPub.publish(thetaZero)
    rightSteerPub.publish(thetaOne)
    frontLeftVelPub.publish(inputVelocity)
    frontRightVelPub.publish(inputVelocity)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("ackermann_cmd", AckermannDrive, pubSteer)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

#!/usr/bin/env python

import rospy
import tf
import roslib
import random 
import math
import copy as copy_module
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from ackermann_msgs.msg import AckermannDrive
from math import pi

class CommandEncoder:
    def __init__(self, targetFrameRate):
        rospy.init_node('odometry_broadcaster')
        self.targetFrameRate = targetFrameRate
        self.commands = rospy.Subscriber('joint_states', JointState, self.stateSetter)
        self.lastData = JointState()
        self.lastCommandTime = rospy.Time.now().to_sec()
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.br = tf.TransformBroadcaster()
        self.lengthBetweenWheelBase = 0.4 * 3.4 * 0.7
        self.width = 0.8
        self.yaw = 0
        self.x = 0
        self.y = 0
        self.z = 0.2
        self.wheelLength = 2 * pi * 0.2

    def stateSetter(self, states):
        self.lastData = states

    def provider(self):
        iteration = 0
        r = rospy.Rate(self.targetFrameRate)
        while not rospy.is_shutdown():
            if len(self.lastData.name) > 1:
                timeDerevative = rospy.Time.now().to_sec() - self.lastCommandTime
                lastData = copy_module.deepcopy(self.lastData)
                velocityIndex = lastData.name.index('front_left_wheel_joint')
                leftSteeringIndex = lastData.name.index('front_left_steering_knuckle_joint')
                rightSteeringIndx = lastData.name.index('front_right_steering_knuckle_joint')
                velocity = (lastData.velocity[velocityIndex] / (2 * pi)) * self.wheelLength
                leftSteering = lastData.position[leftSteeringIndex]
                rightSteering = lastData.position[rightSteeringIndx]
                maxSteering = max(abs(leftSteering), abs(rightSteering))
                angle = 0
                try:
                    angle = math.atan(self.lengthBetweenWheelBase / ((self.lengthBetweenWheelBase / math.tan(maxSteering)) + (self.width / 2)))
                except:
                    angle = 0
                if leftSteering < 0 : 
                    angle = angle * -1
            
                self.x = self.x + (timeDerevative * velocity) * math.cos(self.yaw)
                self.y = self.y + (timeDerevative * velocity) * math.sin(self.yaw)
                angular_speed = math.atan(angle) * velocity / 0.952
                self.yaw = self.yaw + (timeDerevative * velocity / self.lengthBetweenWheelBase) * math.tan(angle)
                o = quaternion_from_euler(0, 0, self.yaw)
            
            
            
                odom = Odometry()
                odom.header.frame_id = "odom"
                odom.pose.pose = Pose(Point(self.x, self.y, self.z), Quaternion(o[0], o[1], o[2], o[3]))
                odom.child_frame_id = "base_link"
                odom.twist.twist = Twist(Vector3(velocity, 0, 0), Vector3(0, 0, angular_speed))

                # publish the message
                
                sendTime = rospy.Time.now()
                odom.header.stamp = sendTime
                self.br.sendTransform((self.x, self.y, 0), (0, 0, 0, 1), sendTime, 'base_footprint', 'odom')
                self.br.sendTransform((0, 0, self.z), (0, 0, 0, 1), sendTime, 'base_stabilized', 'base_footprint')
                self.br.sendTransform((0, 0, 0), (o[0], o[1], o[2], o[3]), sendTime, 'base_link', 'base_stabilized')
                self.odom_pub.publish(odom)
                iteration += 1
            self.lastCommandTime = rospy.Time.now().to_sec()
            r.sleep()
if __name__ == '__main__':
    commandManager = CommandEncoder(100)
    commandManager.provider()

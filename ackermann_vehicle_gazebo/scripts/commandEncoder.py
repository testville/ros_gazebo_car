#!/usr/bin/env python

import rospy
import tf
import roslib
import random 
import math
import copy as copy_module
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from ackermann_msgs.msg import AckermannDrive

class CommandEncoder:
    def __init__(self, targetFrameRate):
        rospy.init_node('odometry_broadcaster')
        self.targetFrameRate = targetFrameRate
        self.commands = rospy.Subscriber('ackermann_cmd', AckermannDrive, self.commandEncoder)
        self.lastPose = Pose()
        self.lastPose.position = Point(0.0001, 0.0001, 0.0001)
        self.lastPose.orientation = Quaternion(0.0001, 0, 0, 1.0)
        self.lastPose.orientation.x
        self.lastCommand = AckermannDrive()
        self.lengthBetweenWheelBase = 0.4 * 3.4 * 0.7
        self.Timer = rospy.Time
        self.lastCommandTime = self.Timer.now().to_sec()
        self.initialZPose = 0.2
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

    def commandEncoder(self, command):
        self.lastCommand = command

    def provider(self):
        iteration = 0
        r = rospy.Rate(self.targetFrameRate)
        while not rospy.is_shutdown():
            timeDerevative = rospy.Time.now().to_sec() - self.lastCommandTime
            command = copy_module.deepcopy(self.lastCommand)
            currentYaw = euler_from_quaternion((self.lastPose.orientation.x, self.lastPose.orientation.y, self.lastPose.orientation.z, self.lastPose.orientation.w))[2]
            self.lastPose.position.x = self.lastPose.position.x + (timeDerevative * command.speed) * math.cos(currentYaw)
            self.lastPose.position.y = self.lastPose.position.y + (timeDerevative * command.speed) * math.sin(currentYaw)
            angular_speed = math.atan(command.steering_angle) * command.speed / 0.952
            o = quaternion_from_euler(0, 0, currentYaw + (timeDerevative * command.speed / self.lengthBetweenWheelBase) * math.tan(command.steering_angle))
                
            self.lastPose.orientation = Quaternion(o[0], o[1], o[2], o[3])
            br = tf.TransformBroadcaster()
            p = self.lastPose.position
            
            
            
            odom = Odometry()
            
            odom.header.frame_id = "odom"
            odom.pose.pose = Pose(Point(p.x, p.y, p.z), Quaternion(o[0], o[1], o[2], o[3]))
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(self.lastCommand.speed, 0, 0), Vector3(0, 0, angular_speed))

            # publish the message
            self.odom_pub.publish(odom)
            self.lastCommandTime = rospy.Time.now().to_sec()
            sendTime = rospy.Time.now()
            odom.header.stamp = self.lastCommandTime
            br.sendTransform((p.x, p.y, 0), (0, 0, 0, 1), sendTime, 'base_footprint', 'odom')
            br.sendTransform((0, 0, self.initialZPose), (0, 0, 0, 1), sendTime, 'base_stabilized', 'base_footprint')
            br.sendTransform((0, 0, 0), (o[0], o[1], o[2], o[3]), sendTime, 'base_link', 'base_stabilized')
            
            iteration += 1
            r.sleep()
if __name__ == '__main__':
    rospy.loginfo('we are starting')
    commandManager = CommandEncoder(300)
    commandManager.provider()

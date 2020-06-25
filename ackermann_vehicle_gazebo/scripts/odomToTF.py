#!/usr/bin/env python

import rospy
import tf
import roslib
import random 

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def getOdomData(msgs):
    position = msgs.pose.pose.position
    orientation =  msgs.pose.pose.orientation
    br = tf.TransformBroadcaster()
    br.sendTransform((position.x, position.y, 0), (0, 0, 0, 1), rospy.Time.now(), 'base_footprint', 'odom')
    br.sendTransform((0, 0, position.z), (0, 0, 0, 1), rospy.Time.now(), 'base_stabilized', 'base_footprint')
    br.sendTransform((0, 0, 0), (orientation.x, orientation.y, orientation.z, orientation.w), rospy.Time.now(), 'base_link', 'base_stabilized')

 
rospy.init_node('odometry_broadcaster')
odom_sub = rospy.Subscriber("odom_truth", Odometry, getOdomData)
rospy.spin()


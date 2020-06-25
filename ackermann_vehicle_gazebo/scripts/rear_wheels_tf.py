#!/usr/bin/env python
import rospy

# to get commandline arguments
import sys

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
        
        rospy.init_node('my_static_tf2_broadcaster', log_level=rospy.ERROR)
        rospy.logerr("returned the invalid value %s %s %s %s %s", sys.argv[0], sys.argv[1], sys.argv[2] , sys.argv[8] , sys.argv[9])
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        
        static_transformStamped.transform.translation.x = float(sys.argv[1])
        static_transformStamped.transform.translation.y = float(sys.argv[2])
        static_transformStamped.transform.translation.z = float(sys.argv[3])

        static_transformStamped.transform.rotation.x = float(sys.argv[4])
        static_transformStamped.transform.rotation.y = float(sys.argv[5])
        static_transformStamped.transform.rotation.z = float(sys.argv[6])
        static_transformStamped.transform.rotation.w = float(sys.argv[7])

        static_transformStamped.header.frame_id = sys.argv[8]
        static_transformStamped.child_frame_id = sys.argv[9]

        broadcaster.sendTransform(static_transformStamped)
        rospy.spin()
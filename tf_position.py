#!/usr/bin/env python  
import roslib
roslib.load_manifest('my_demo')
import rospy
import math
import tf
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion

if __name__ == '__main__':
    rospy.init_node('rosky_position')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            (roll, pitch, raw) = euler_from_quaternion( rot )
            degree = raw*180.0/math.pi
            if ( degree < 0 ) :
              degree = degree+360
            rospy.loginfo( trans )
            rospy.loginfo( [ roll, pitch, raw ] )
            rospy.loginfo( degree )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

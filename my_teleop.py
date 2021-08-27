#!/usr/bin/env python

from __future__ import print_function

import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

def speedtest():
  global cmd_vel_pub

  twist_msg.linear.x = 0  
  twist_msg.linear.y = 0
  twist_msg.linear.z = 0
  twist_msg.angular.x = 0
  twist_msg.angular.y = 0
  twist_msg.angular.z = 0
  cmd_vel_pub.publish( twist_msg )

  for j in range( 0 ) :
    for i in range( 0 ) :
      twist_msg.linear.x =  0.22 # 1000/3*x^2+110/3*x = y
      print( twist_msg.linear.x )
      twist_msg.linear.y = 0 * 0.5
      twist_msg.linear.z = 0 * 0.5
      twist_msg.angular.x = 0
      twist_msg.angular.y = 0
      twist_msg.angular.z = 0 * 1
      rospy.loginfo( "go go" )
      cmd_vel_pub.publish( twist_msg ) # forward
      rospy.sleep( 0.6 )

    # end for
    """
    twist_msg.linear.x = 1 * 0.5
    twist_msg.linear.y = 0 * 0.5
    twist_msg.linear.z = 0 * 0.5
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = 0 * 1
    cmd_vel_pub.publish( twist_msg ) # forward
    rospy.sleep( 0.6 )
    """

    for i in range( 2 ):
      twist_msg.linear.x = 0 * 0.5
      twist_msg.linear.y = 0 * 0.5
      twist_msg.linear.z = 0 * 0.5
      twist_msg.angular.x = 0
      twist_msg.angular.y = 0
      twist_msg.angular.z = 1 * 0.166
      cmd_vel_pub.publish( twist_msg ) # left turn
      rospy.sleep( 1 ) 

  # end for

  for i in range( 2 ) :
    twist_msg.linear.x = 0 * 0.5
    twist_msg.linear.y = 0 * 0.5
    twist_msg.linear.z = 0 * 0.5
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = 0.1777 * 1  # 3000*x^2+-280*x 
    cmd_vel_pub.publish( twist_msg ) # forword
    rospy.sleep( 1 )
  # end for
  
  """
  twist_msg.linear.x = 1 * 0.5
  twist_msg.linear.y = 0 * 0.5
  twist_msg.linear.z = 0 * 0.5
  twist_msg.angular.x = 0
  twist_msg.angular.y = 0
  twist_msg.angular.z = -1 * 1
  cmd_vel_pub.publish( twist_msg ) # right turn
  rospy.sleep( 0.6 )
  twist_msg.linear.x = -1 * 0.5
  twist_msg.linear.y = 0 * 0.5
  twist_msg.linear.z = 0 * 0.5
  twist_msg.angular.x = 0
  twist_msg.angular.y = 0
  twist_msg.angular.z = 0 * 1
  cmd_vel_pub.publish( twist_msg ) # back
  rospy.sleep( 0.6 )
  twist_msg.linear.x = 1 * 0.5
  twist_msg.linear.y = 0 * 0.5
  twist_msg.linear.z = 0 * 0.5
  twist_msg.angular.x = 0
  twist_msg.angular.y = 0
  twist_msg.angular.z = 1 * 1
  cmd_vel_pub.publish( twist_msg ) # left turn
  rospy.sleep( 0.6 )
  """ 
  twist_msg.linear.x = 0
  twist_msg.linear.y = 0
  twist_msg.linear.z = 0
  twist_msg.angular.x = 0
  twist_msg.angular.y = 0
  twist_msg.angular.z = 0
  cmd_vel_pub.publish( twist_msg )

# end speedtest()

if __name__ == '__main__':
  rospy.init_node( 'move_controller' )
  rospy.loginfo( "controller on" )
  rospy.sleep( 10 ) # wait for lidar
  # cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
  twist_msg = Twist()

  """
  while( True ) :
    twist_msg.linear.x = x * speed
    twist_msg.linear.y = y * speed
    twist_msg.linear.z = z * speed
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = th * turn
    cmd_vel_p-ub.publish( twist_msg )
  # while end
  """

  speedtest()

# end main

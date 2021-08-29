#!/usr/bin/env python

from __future__ import print_function

import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

from sympy import *

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
twist_msg = Twist()

def calculate_linear( length ) :
  x = Symbol('x')
  f = 1000/3*x**2+110/3*x-length
  sol = solve( f, x )
  if ( sol[0] > 0 ) :
    sol = sol[0]
  # end if()
  else :
    sol = sol[1]
  # end else()
  
  sol = round( sol , 3 )
  return sol

# end calculate_linear()
  
def calculate_angular( degree ) :
  x = Symbol('x')
  # f = 3000*x**2-280*x-degree
  f = 5000*x**2-550*x-degree
  sol = solve( f, x )
  if ( sol[0] > 0 ) :
    sol = sol[0]
  # end if()
  else :
    sol = sol[1]
  # end else()
  
  sol = round( sol , 3 )
  return sol

# end calculate_angular()

def Forward( length ):
  global cmd_vel_pub
  global twist_msg
  print( "hi" )
  while( length > 20 ) :
    twist_msg.linear.x =  0.22 # 1000/3*x^2+110/3*x = y
    twist_msg.linear.y = 0 
    twist_msg.linear.z = 0 
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = 0 
    rospy.loginfo( "go 20" )
    cmd_vel_pub.publish( twist_msg ) # forward
    rospy.sleep( 0.6 )
    length = length-20
  # end while()
  
  
  twist_msg.linear.x =  calculate_linear( length )
  twist_msg.linear.y = 0 
  twist_msg.linear.z = 0 
  twist_msg.angular.x = 0
  twist_msg.angular.y = 0
  twist_msg.angular.z = 0 
  rospy.loginfo( "go 20" )
  cmd_vel_pub.publish( twist_msg ) # forward
  rospy.sleep( 0.6 )
  
# end Forward
      
def Turn_left( degree ):
  global cmd_vel_pub
  global twist_msg
  while( degree >= 360 ) :
    degree = degree-360
  # end while()
  
  while( degree > 75 ) :
    twist_msg.linear.x =  0
    twist_msg.linear.y = 0 
    twist_msg.linear.z = 0 
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = 1
    rospy.loginfo( "go 75" )
    cmd_vel_pub.publish( twist_msg ) 
    rospy.sleep( 1 )
    degree = degree-75
  # end while() 
  
  twist_msg.linear.x =  0
  twist_msg.linear.y = 0 
  twist_msg.linear.z = 0 
  twist_msg.angular.x = 0
  twist_msg.angular.y = 0
  twist_msg.angular.z = calculate_angular( degree ) 
  # twist_msg.angular.z = 0.14
  rospy.loginfo( "go" )
  cmd_vel_pub.publish( twist_msg ) 
  rospy.sleep( 1 )
  
# end Turn_left()

def Turn_right( degree ):
  global cmd_vel_pub
  global twist_msg
  while( degree >= 360 ) :
    degree = degree-360
  # end while()
  
  while( degree > 75 ) :
    twist_msg.linear.x =  0
    twist_msg.linear.y = 0 
    twist_msg.linear.z = 0 
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = -1
    rospy.loginfo( "go 75" )
    cmd_vel_pub.publish( twist_msg ) 
    rospy.sleep( 1 )
    degree = degree-75
  # end while() 
  
  twist_msg.linear.x =  0
  twist_msg.linear.y = 0 
  twist_msg.linear.z = 0 
  twist_msg.angular.x = 0
  twist_msg.angular.y = 0
  twist_msg.angular.z = -( calculate_angular( degree ) )
  rospy.loginfo( "go" )
  cmd_vel_pub.publish( twist_msg ) 
  rospy.sleep( 1 )
  
# end Turn_right()

def Backward( length ):
  global cmd_vel_pub
  global twist_msg
  while( length > 20 ) :
    twist_msg.linear.x =  -0.22 # 1000/3*x^2+110/3*x = y
    twist_msg.linear.y = 0 
    twist_msg.linear.z = 0 
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = 0 
    rospy.loginfo( "go 20" )
    cmd_vel_pub.publish( twist_msg ) # backward
    rospy.sleep( 0.6 )
    length = length-20
  # end while()
  
  
  twist_msg.linear.x =  -( calculate_linear( length ) )
  twist_msg.linear.y = 0 
  twist_msg.linear.z = 0 
  twist_msg.angular.x = 0
  twist_msg.angular.y = 0
  twist_msg.angular.z = 0 
  rospy.loginfo( "go 20" )
  cmd_vel_pub.publish( twist_msg ) # backward
  rospy.sleep( 0.6 )
  
# end Backward()

def speedinit():
  global cmd_vel_pub
  global twist_msg
  twist_msg.linear.x =  0
  twist_msg.linear.y = 0 
  twist_msg.linear.z = 0 
  twist_msg.angular.x = 0
  twist_msg.angular.y = 0
  twist_msg.angular.z = 0 
  cmd_vel_pub.publish( twist_msg )

# end speedinit()

def speedtest():
  # 1000/3*x^2+110/3*x = y
  # 3000*x^2+-280*x
  for i in range( 1 ):
    Forward( 100 )
   # Turn_left( 90 )
    # Turn_left( 45 )
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
  speedinit()
  speedtest()

# end main

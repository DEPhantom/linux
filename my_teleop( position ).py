#!/usr/bin/env python

from __future__ import print_function

import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

from sympy import *

import math

import roslib
import tf_position
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion
import threading

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
twist_msg = Twist()
virtual_x = 0
virtual_y = 0
virtual_degree 0
actual_x = 0
actual_y = 0
actual_degree = 0


def set_init_pose( x, y, degree ) :
  global virtual_x 
  global virtual_y 
  global virtual_degree
  virtual_x = x
  virtual_y = y
  virtual_degree = degree
  # Settings pose
  
# end set_init_pose()

def record_position( length, degree ) :
  global virtual_x 
  global virtual_y 
  global virtual_degree
  virtual_degree = virtual_degree + degree 
  while ( virtual_degree >= 360 ):
    virtual_degree = virtual_degree - 360
  # end while 
  
  while ( virtual_degree < 0 ):
    virtual_degree = virtual_degree + 360
  # end while 
  
  virtual_x = virtual_x + length*math.cos( math.py/180*virtual_degree )
  virtual_y = virtual_y + length*math.sin( math.py/180*virtual_degree )
  
# end record_position()

def update_actual_pose() :
  global actual_x 
  global actual_y 
  global actual_degree 
  
  listener = tf.TransformListener()
  rate = rospy.Rate(10.0)
  while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        (roll, pitch, raw) = euler_from_quaternion( rot )
        degree = raw*180.0/math.pi
        if ( degree < 0 ) :
            degree = degree+360
        actual_x = round( trans[0] , 2 )*100
        actual_y = round( trans[1] , 2 )*100
        actual_degree = round( degree , 0 )
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    rate.sleep()
  
  # end while()  
   
# end update_actual_pose()

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
  record_position( length, 0 )
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
  record_position( 0, degree )
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
  record_position( 0, -degree )
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
  record_position( -length, 0 )
# end Backward()

def adjust_position() :
  global virtual_x 
  global virtual_y 
  global virtual_degree
  current_x = actual_x
  current_y = actual_y
  if ( virtual_degree == 0 || virtual_degree == 180 ) :
    if ( virtual_x - current_x > 10 ) :
      Forward( virtual_x - current_x )
    # end if
    elif ( virtual_x - current_x < -10 ) :
      Backward( -(virtual_x - current_x) ) 
    # end elif
    
  # end if
  elif ( virtual_degree == 90 || virtual_degree == 270 ) :
    if ( virtual_y - current_y > 10 ) :
      Forward( virtual_y - current_y )
    # end if
    elif ( virtual_y - current_y < -10 ) :
      Backward( -(virtual_y - current_y) ) 
    # end elif
    
  # end elif
  
# end adjust_position()
  
def adjust_degree() :
  global virtual_degree
  current_degree = actual_degree
  if ( virtual_degree - current_degree > 10 ) :
    Turn_left( virtual_degree - current_degree )
  # end if
  elif ( virtual_degree - current_degree < -10 ) :
    Turn_right( -(virtual_degree - current_degree) ) 
  # end elif 
  
# end adjust_degree()

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
  set_init_pose( 0, 0, 0 )
  for i in range( 4 ):
    Forward( 160 )
    adjust_position()
    Turn_left( 95 )
    adjust_degree()
  # end for 
  
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
  thread1 = threading.Thread(target = update_actual_pose() )
  thread1.start()
  speedinit()
  speedtest()
  
  thread1.join()

# end main
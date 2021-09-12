#!/usr/bin/env python

from __future__ import print_function

import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

from sympy import *

import math

import roslib
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import threading
import time

import pathxy
import AStar
import map2d

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
twist_msg = Twist()
virtual_x = 0
virtual_y = 0
virtual_degree =  0
actual_x = 0
actual_y = 0
actual_degree = 0
path_count = 0
end = False

class pose:
  
  def __init__(self):
    self.direct = ""
    self.degree = 0
    self.distance = 0
  # end pose()
  
# end class

def set_init_pose( x, y, degree ) :
  global virtual_x 
  global virtual_y 
  global virtual_degree
  virtual_x = x
  virtual_y = y
  virtual_degree = degree
  pub = rospy.Publisher( "/initialpose", PoseWithCovarianceStamped, queue_size=10 )
  initpose_msg = PoseWithCovarianceStamped()
  initpose_msg.header.frame_id = "map"
  initpose_msg.pose.pose.position.x = x
  initpose_msg.pose.pose.position.y = y
  initpose_msg.pose.pose.position.z = 0.06
  quaternion = quaternion_from_euler( 0, 0, degree )
  initpose_msg.pose.pose.orientation.w = quaternion[0]
  initpose_msg.pose.pose.orientation.w = quaternion[1]
  initpose_msg.pose.pose.orientation.w = quaternion[2]
  initpose_msg.pose.pose.orientation.w = quaternion[3]
  rospy.loginfo( "setting initial position" )
  for i in range( 3 ): # first will miss
    pub.publish( initpose_msg )
    rospy.sleep( 0.5 ) # too fast will miss
  
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
 
  print( "before : %d, %d" %( virtual_x, virtual_degree) ) 
  virtual_x = virtual_x + length*math.cos( math.pi/180*virtual_degree )
  virtual_y = virtual_y + length*math.sin( math.pi/180*virtual_degree )
  print( "after : %d, %d" %( virtual_x, virtual_degree) )  

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
  # f = 5000*x**2-550*x-degree
  f = 35000/17*x**2-38.24*x-degree 
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

def Forward( length, is_adjust ):
  global cmd_vel_pub
  global twist_msg
  if ( not is_adjust ) :
    record_position( length, 0 )
  
  start = time.time()
  end = time.time()
  run_time = length*3/71
  while( end - start <= run_time ) :
    twist_msg.linear.x =  0.13
    twist_msg.linear.y = 0 
    twist_msg.linear.z = 0 
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = 0 
    cmd_vel_pub.publish( twist_msg ) # forward
    end = time.time()
  # end while()
  
  for i in range( 20 ) :
    twist_msg.linear.x = 0
    twist_msg.linear.y = 0 
    twist_msg.linear.z = 0 
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = 0 
    cmd_vel_pub.publish( twist_msg ) # stop
	rospy.sleep( 0.1 ) # too fast will miss
  # end for

# end Forward
      
def Turn_left( degree, is_adjust ):
  global cmd_vel_pub
  global twist_msg
  while( degree >= 360 ) :
    degree = degree-360
  # end while()
  
  if ( not is_adjust ) :
    record_position( 0, degree )

  while( degree > 70 ) :
    twist_msg.linear.x =  0
    twist_msg.linear.y = 0 
    twist_msg.linear.z = 0 
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = 1
    rospy.loginfo( "turn left 70" )
    cmd_vel_pub.publish( twist_msg ) 
    rospy.sleep( 1 )
    degree = degree-70
  # end while() 
  
  twist_msg.linear.x =  0
  twist_msg.linear.y = 0 
  twist_msg.linear.z = 0 
  twist_msg.angular.x = 0
  twist_msg.angular.y = 0
  twist_msg.angular.z = calculate_angular( degree ) 
  # twist_msg.angular.z = 0.14
  rospy.loginfo( "turn left %d" %degree )
  cmd_vel_pub.publish( twist_msg ) 
  rospy.sleep( 1 )

# end Turn_left()

def Turn_right( degree, is_adjust ):
  global cmd_vel_pub
  global twist_msg
  while( degree >= 360 ) :
    degree = degree-360
  # end while()
  
  if ( not is_adjust ) :
    record_position( 0, -degree )

  while( degree > 70 ) :
    twist_msg.linear.x =  0
    twist_msg.linear.y = 0 
    twist_msg.linear.z = 0 
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = -1
    rospy.loginfo( "turn right 70" )
    cmd_vel_pub.publish( twist_msg ) 
    rospy.sleep( 1 )
    degree = degree-70
  # end while() 
  
  twist_msg.linear.x =  0
  twist_msg.linear.y = 0 
  twist_msg.linear.z = 0 
  twist_msg.angular.x = 0
  twist_msg.angular.y = 0
  twist_msg.angular.z = -( calculate_angular( degree ) )
  rospy.loginfo( "turn right %d" %degree )
  cmd_vel_pub.publish( twist_msg ) 
  rospy.sleep( 1 )

# end Turn_right()

def Backward( length, is_adjust ):
  global cmd_vel_pub
  global twist_msg
  if ( not is_adjust ) :
    record_position( -length, 0 )

  start = time.time()
  end = time.time()
  run_time = length*3/71
  while( end - start <= run_time ) :
    twist_msg.linear.x = -0.13
    twist_msg.linear.y = 0 
    twist_msg.linear.z = 0 
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = 0 
    cmd_vel_pub.publish( twist_msg ) # back
    end = time.time()
  # end while()
  
  for i in range( 20 ) :
    twist_msg.linear.x = 0
    twist_msg.linear.y = 0 
    twist_msg.linear.z = 0 
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = 0 
    cmd_vel_pub.publish( twist_msg ) # stop
    rospy.sleep( 0.1 ) # too fast will miss
  # end for

# end Backward()

def Forward_ad( length, is_adjust ):
  global cmd_vel_pub
  global twist_msg
  if ( not is_adjust ) :
    record_position( length, 0 )

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
  rospy.loginfo( "go %d" %length )
  cmd_vel_pub.publish( twist_msg ) # forward
  rospy.sleep( 0.6 )

# end Forward_ad

def adjust_position() :
  global virtual_x 
  global virtual_y 
  global virtual_degree
  global actual_x
  global actual_y

  rospy.sleep( 0.5 )
  current_x = actual_x
  current_y = actual_y

  rospy.loginfo( "adjust linear" )
  print( "virtual is %d, %d" %(virtual_x, virtual_y))
  print( "actual is %d, %d" %(current_x, current_y))

  if ( virtual_degree == 0 or virtual_degree == 180 ) : # watch x
    distance = virtual_x*math.cos( virtual_degree ) - actual_x*math.cos( virtual_degree )
    if ( distance > 10 ) :
      print( "go %d" %( virtual_x - current_x ) )
      Forward_ad( distance, True )
    # end if
    elif ( distance < -10 ) :
      print( "back %d" %( virtual_x - current_x ) )
      Backward( -distance, True ) 
    # end elif
    
  # end if
  elif ( virtual_degree == 90 or virtual_degree == 270 ) : # watch y
    distance = virtual_y*math.sin( virtual_degree ) - actual_y*math.sin( virtual_degree )
    if ( distance > 10 ) :
      Forward_ad( distance, True )
    # end if
    elif ( distance < -10 ) :
      Backward( -distance, True ) 
    # end elif
    
  # end elif
  
# end adjust_position()
  
def adjust_degree() :
  global virtual_degree
  global actual_degree

  rospy.sleep( 0.5 )
  current_degree = actual_degree
  rospy.loginfo( "adjust angular" )
  print( "virtual is %d" %(virtual_degree))
  print( "actual is %d" %(current_degree))

  if ( virtual_degree - current_degree > 180 ) :
    Turn_right( 360 - virtual_degree - current_degree, True )
  # end if
  elif ( virtual_degree - current_degree > 5 ) :
    Turn_left( virtual_degree - current_degree, True )
  elif ( virtual_degree - current_degree < -180 ) :
    Turn_left( ( virtual_degree - current_degree + 360 ), True ) 
  elif ( virtual_degree - current_degree < -5 ) :
    Turn_right( -( virtual_degree - current_degree), True ) 
  # end elif 
  
# end adjust_degree()

def find_degree( path ) :
  global path_count
  x = path[path_count+1].x - path[path_count].x
  y = path[path_count+1].y - path[path_count].y
  if ( x == 0 and y == 1 ) :
    return 90
  elif ( x == 1 and y == 1 ) :
    return 45
  elif ( x == 1 and y == 0 ) :
    return 0
  elif ( x == 1 and y == -1 ) :
    return 315
  elif ( x == 0 and y == -1 ) :
    return 270
  elif ( x == -1 and y == -1 ) :
    return 225
  elif ( x == -1 and y == 0 ) :
    return 180
  elif ( x == -1 and y == 1 ) :
    return 135
  else :
    print( "error" )
	
# end find_degree()

def find_direct( now_degree, future_degree ) :
  if ( now_degree > future_degree ) :
    return "right"
  elif ( now_degree < future_degree ):
    return "left"
  else:
    return "forward"
	
# end find_direct()

def part_path( path ) :
  global end
  global path_count
  global virtual_degree
  
  if ( len( path ) > 1 and ( virtual_degree == find_degree() ) ) :
    now_degree = virtual_degree
	path_count = path_count+1
    distance = 1
  # end if()
  else :
	now_degree = find_degree()
	distance = 0
  # end else
  
  while ( ( path_count < len( path )-1 ) and find_degree() == virtual_degree ) :
    distance = distance + 1
	path_count = path_count+1
  # end while()
  
  if ( path_count == len( path )-1 or len( path ) == 0 ) :
    end = True
	
  action = pose()
  action.direct = find_direct( virtual_degree, now_degree )
  action.degree = abs( virtual_degree - now_degree ) 
  action.distance = distance*5 #5cm
  return action
	
# end part_path()

def speedinit():
  global cmd_vel_pub
  global twist_msg
  
  for i in range( 3 ) :
    twist_msg.linear.x =  0
    twist_msg.linear.y = 0 
    twist_msg.linear.z = 0 
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = 0 
    cmd_vel_pub.publish( twist_msg )
    rospy.sleep( 0.5 ) # too fast will miss
  # end for
  
# end speedinit()

def navigation() :
  global end
  global virtual_degree
  
  # build map
  aStar_planner = map2d.map2d()
  aStar_planner.getMap()
  aStar_planner.get_origin()
  aStar_planner.pathTag = 'o'
  # setting origin and goal
  aStar = AStar.AStar(aStar_planner, AStar.Node(AStar.Point(aStar_planner.origin_x, aStar_planner.origin_y)), 
                      AStar.Node(AStar.Point(1, 1)))

  # find way
  if aStar.start():
    aStar.setMap()
    aStar_planner.get_path(aStar.pathlist) 
	
  path = aStar_planner.pathway

  while ( not end ):
    action = part_path( path ) 
    if ( action.direct == "forward" ) :
	  Forward( action.distance )
      for i in range( 3 ) :
        adjust_position()
      # end for
      
    # end if
    elif ( action.direct == "left" ) :
	  Turn_left( action.degree )
      for i in range( 3 ) :
        adjust_degree()
      # end for
      
    # end if
    elif ( action.direct == "right" ) :
	  Turn_right( action.degree )
      for i in range( 3 ) :
        adjust_degree()
      # end for
      
    # end if
    elif ( action.direct == "back" ) :
	  Backward( action.distance )
      for i in range( 3 ) :
        adjust_position()
      # end for
      
    # end if
    else :
	  loginfo( "diretion is %s" %action.direct )
	  
  # end while()
	  
# end navigation()

if __name__ == '__main__':
  rospy.init_node( 'global planner' )
  rospy.loginfo( "global planner on" )
  rospy.sleep( 10 ) # wait for lidar
  thread1 = threading.Thread( target = update_actual_pose )
  thread1.start()
  speedinit()
  navigation()

  thread1.join()  

# end main

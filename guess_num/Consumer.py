#!/usr/bin/python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from random import randint

guess_num = -1
has_join = False
end = False
pub = rospy.Publisher( "answer", String, queue_size = 10 )

def callback( data ):
  global has_join
  print( "Producer is join" )
  has_join = True

def ready():
  global guess_num, has_join
  rospy.Subscriber( "join", String, callback )
  while ( has_join == False ) :
    rospy.sleep(0.01)
  # end while()

  guess_num = randint( 0, 100 )

# end ready()

def check_num( data ):
  global guess_num, end, pub

  if ( data.data > guess_num ):
    ans_str = "Lower"
  elif ( data.data == guess_num ):
    ans_str = "Correct"
    end = True
  # end elif
  else:
    ans_str = "Higher"

  rospy.loginfo( ans_str )
  pub.publish( ans_str )

# end checknum

def start():
  global result, pub, end, guess_num

  end = False
  rospy.Subscriber( "guess", Int16, check_num )
  rospy.loginfo( "please guess a number" )
  pub.publish( "please guess a number" )
  while ( not end ):
    rospy.sleep(0.001)

  # end while()

  print( "Answer is %d" %guess_num )

# end start()

def main():
  rospy.init_node( "Consumer", anonymous = True )
  ready()
  start()

# end main()

if __name__ == '__main__':
  main()
# end if

#!/usr/bin/python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from random import randint

status = 0
guess_num = -1
end = False
pub = rospy.Publisher( "join", String, queue_size = 10 )
pub2 = rospy.Publisher( "guess", Int16, queue_size = 10 )

def change( data ):
  global status
  if ( data.data == "please guess a number" ):
    status = 1

# end change()

def standby( event ):
  global pub
  rospy.loginfo( "I'm ready" )
  pub.publish( "I'm ready" )
# end standby

def ready():
  global status, guess_num
  sub = rospy.Subscriber( "answer", String, change )
  timer = rospy.Timer( rospy.Duration(0.5), standby )

  while( status == 0 ):
    rospy.sleep(0.01)

  timer.shutdown()
  # sub.unregister()
  guess_num = randint( 0, 100 )

# end ready()

def change_num( data ):
  global end, guess_num, pub2
  if ( data.data == "Higher" ):
    guess_num = guess_num+1
  elif ( data.data == "Lower" ):
    guess_num = guess_num//2
  else:
    end = True

  rospy.loginfo( guess_num )
  pub2.publish( guess_num )

# end change_num()

def start():
  global pub2, guess_num, end
  end = False
  rospy.Subscriber( "answer", String, change_num )
  rospy.loginfo( guess_num )
  pub2.publish( guess_num )
  while ( not end ):
    rospy.sleep(0.001)

  # end while()

# end start()

def main():
  rospy.init_node( "Producer", anonymous = True )

  ready()
  start()

# end main()

if __name__ == '__main__':
  main()
# end if

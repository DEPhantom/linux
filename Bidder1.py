#!/usr/bin/python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from random import randint

end = False
money = "100"
limit = "300"

pub = rospy.Publisher( "join", String, queue_size = 10 )
pub2 = rospy.Publisher( "bid1", Int16, queue_size = 10 )

def check_price( data ):
  global money
  money = data.data
  end = True
# end check_price()

def standby():
  global pub
  pub.publish( "I'm ready" )

# end standby()

def ready():
  global end
  rospy.Subsciber( "join1", String, check_price )
  timer = rospy.Timer( rospy.Duration( 0.5 ), standby )
  while( end == False ):
    rospy.sleep(1)
  # end while()
  
  timer.shutdown()
  
# end ready()

def add_price( data ):
  global winner, money, limit, end
  
  if ( data.data > money and data.data < limit ):
    money = data.data+randint( 1, 10 )
	rospy.loginfo( "Bidder1: %d" %money )
    pub2.publish( money )
  # end if
  
# end add_price

def start():
  global end, winner, money, pub
  end = False
  money = money+randint( 1, 10 )
  rospy.loginfo( "Bidder1: %d" %money )
  pub2.publish( money )
  rospy.Subsciber( "bid1", Int16, add_price )
  while( not end ):
    rospy.sleep( 0.001 )
  # end while()
  
# end start()

def main():
  rospy.init_node( "Bidder1", anonymous = True )
  ready()
  start()
  
# end main()

if __name__ == "__main__":
  main()
# end if





















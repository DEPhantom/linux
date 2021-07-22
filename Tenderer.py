#!/usr/bin/python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16

has_join1 = False
has_join2 = False
has_join3 = False
end = False
money = 100
target = 300
winner = ""

pub = rospy.Publisher( "price", Int16, queue_size = 10 )

def check1( data ):
  global has_join1
  rospy.loginfo( "Bidder1 is join" )
  has_join1 = True
# end check1()

def check2( data ):
  global has_join2
  rospy.loginfo( "Bidder2 is join" )
  has_join2 = True
# end check2()

def check3( data ):
  global has_join3
  rospy.loginfo( "Bidder3 is join" )
  has_join3 = True
# end check3()

def ready():
  global has_join1, has_join2, has_join3
  rospy.Subscriber( "join1", String, check1 )
  rospy.Subscriber( "join2", String, check2 )
  rospy.Subscriber( "join3", String, check3 )
  while( has_join1 == False or has_join2 == False or has_join3 == False ):
    rospy.sleep(1)
  # end while()
  
  rospy.loginfo( "Now start bidding" )
  
# end ready()

def check_num1( data ):
  global winner, money, target, end, pub
  
  if ( data.data > money ):
    money = data.data
    rospy.loginfo( "Leader is bidder1, price is "+  str(money) )
  # end if
  
  if ( money >= target ):
    end = True
    winner = "bidder1"
    pub.publish( -1 )
  # end if
  else:
    pub.publish( money )
  # end else()
  
# end check_num1

def check_num2( data ):
  global winner, money, target, end, pub
  
  if ( data.data > money ):
    money = data.data
    rospy.loginfo( "Leader is bidder2, price is :"+ str(money) )
  # end if
  
  if ( money >= target ):
    end = True
    winner = "bidder2"
    pub.publish( -1 )
  # end if
  else:
    pub.publish( money )
  # end else()

# end check_num2

def check_num3( data ):
  global winner, money, target, end, pub
  
  if ( data.data > money ):
    money = data.data
    rospy.loginfo( "Leader is bidder3, price is "+  str(money) )
  # end if
  
  if ( money >= target ):
    end = True
    winner = "bidder3"
    pub.publish( -1 )
  # end if
  else:
    pub.publish( money )
  # end else()

# end check_num3

def start():
  global end, winner, money, pub
  end = False
  rospy.loginfo( "start from " + str(money) )
  pub.publish( money )
  rospy.Subscriber( "bid1", Int16, check_num1 )
  rospy.Subscriber( "bid2", Int16, check_num2 )
  rospy.Subscriber( "bid3", Int16, check_num3 )
  while( not end ):
    rospy.sleep( 0.001 )
  # end while()
  
  rospy.loginfo( "%s use %d get the item" %( winner, money ) )
  
# end start()

def main():
  rospy.init_node( "Tenderer", anonymous = True )
  ready()
  start()
  
# end main()

if __name__ == "__main__":
  main()
# end if





















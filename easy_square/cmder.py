#!/usr/bin/env python


import rospy
import time
from std_msgs.msg import String

pub = rospy.Publisher('pimir02/Command', String, queue_size=10)

def keyboard(data, k):
    global pub 
    start = time.time()
    end = time.time()
    while( end -start <= k ):
        rospy.loginfo(data)
        pub.publish(data)
        end = time.time()


def talker():
    rospy.init_node('talker', anonymous=True)
    initial()
    keyboard("Forward", 1)
    keyboard("Stop", 1 )
    keyboard("Backward", 1)
    keyboard("Stop", 1 )
    keyboard("Left", 0.3 )
    keyboard("Stop", 1 )
    keyboard("Right", 0.3 )
    keyboard("Stop", 1 )


def initial():
    global pub
    for i in range (3) :
        pub.publish("Stop")
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

#! /usr/bin/python
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdlib.h>

std::string r_str ;
ros::Publisher hint_pub ;  // pub = rospy.Publisher('Hint', String, queue_size=10)   
int secret = std::experimental::randint(0,10000) ;                      
void Callback2(const std_msgs::Int16::ConstPtr& number) {
    if( secret > number->data ) {
        r_str = "Higher" ;
        LOG_INFO(r_str) ;
        hint_pub.publish(r_str) ; 
    } // if()
    else if( secret < number->data ) {
        r_str = "Lower" ;
        LOG_INFO(r_str) ;
        hint_pub.publish(r_str) ; 
    } // else if()
    else {
        r_str = "Correct" ;
        LOG_INFO(r_str) ;
        hint_pub.publish(r_str) ;
    } // else()
} // callback2()

void listener() {
    ros::Subscriber number_sub = knower.subscribe("number", 10, Callback2); 
    rospy.spin()
} // listener()

int main(int argc, char ** argv) {
    ros::init(argc,agrv, "knower") ;
    ros::NodeHandle knower ;
    hint_pub = knower.advertise<std_msgs::String>("Hint", 10) ;
    Guesser() ;
} // main()

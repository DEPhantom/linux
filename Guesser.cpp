#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
ros::Publisher number_pub ; 
int min = 0 ;
int max = 10000 ;
int number = rand() % (max - min + 1) + min ; // std::experimental::randint(min,max) ;
std_msgs::Int32 answer ;
void Callback (const std_msgs::String::ConstPtr& msg) {
    if ( strcmp( msg->data.c_str(), "Higher" ) == 0 ) {
        min = number ;
        number = rand() % (max - min + 1) + min ; 
        ROS_INFO("%d",number) ;
        answer.data = number ;
        number_pub.publish(answer.data) ; 
    } // if
    else if ( strcmp( msg->data.c_str(), "Lower") == 0 ) {
        max = number ;
        number = rand() % (max - min + 1) + min ;
        ROS_INFO("%d",number) ;
        answer.data = number ;
        number_pub.publish(answer.data) ; 
    } // else if
    else if ( strcmp( msg->data.c_str(), "Correct") == 0 ) {
        answer.data = number ;
        number_pub.publish(answer.data) ; 
        ROS_INFO("The Answer is: [%d]" , number ) ;
    } // else if
    else 
        answer.data = number ;
        number_pub.publish(answer.data) ;
}
void guesser(){
    ros::Rate loop_rate(1) ;
    ros::NodeHandle G ;
    ros::Subscriber hint_sub = G.subscribe("Hint", 10, Callback) ; 
    while( ros::ok() ) {
        ROS_INFO("%d",number) ;
        answer.data = number ;
        number_pub.publish(answer.data) ; 
        ros::spinOnce() ;  
    } // while()
}

int main(int argc, char ** argv) {
    ros::init(argc,argv, "guesser") ;
    ros::NodeHandle G ;
    number_pub = G.advertise<std_msgs::Int32>("number", 10);
    guesser() ;
} // main()

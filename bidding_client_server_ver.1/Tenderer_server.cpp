# include <ros/ros.h>
# include <my_demo/bid_price.h>
# include <std_msgs/Int32.h>
# include <stdio.h>
# include <stdlib.h>
# include <time.h>
# include <string.h>

ros::Publisher price_pub ;
int target = 0;
int now_price = 100;
std::string leader = "";

bool update( my_demo::bid_price::Request &req,
             my_demo::bid_price::Response &res ) {
  std_msgs::Int32 msg;

  if ( now_price >= target ) {
    ROS_INFO( "Bidding is end, the winner is %s : %d", leader.c_str(),
               now_price );
    res.now_price = 0;
  } // if()
  else if ( req.price > now_price ) {
    now_price = req.price;
    leader = req.leader;
    ROS_INFO( "Now leader is %s : %d", req.leader.c_str(), now_price );
    msg.data = now_price;
    
    for ( int i = 0; i < 2 ; i++ ) { 
      ros::Duration(1).sleep();
      price_pub.publish( msg );
    } // for

    res.now_price = now_price;
  } // else if()
  else {
    ROS_INFO( "Now leader is %s : %d", leader.c_str(), now_price );
    msg.data = now_price;
    ros::Duration(1).sleep();
    price_pub.publish( msg );
    res.now_price = now_price;
  } // else

  return true;

} // update

int main( int argc, char **argv ) {
  std_msgs::Int32 msg;
  ros::init( argc, argv, "Tenderer_server" );
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService( "Bidding", update );
  // srand( time( 0 ) );
  // rand()%5;
  target = 300;
  ROS_INFO( "Now start bidding" );
  price_pub = n.advertise<std_msgs::Int32>( "price", 1000);
  msg.data = now_price;
  
  for ( int i = 0; i < 5 ; i++ ) {
    price_pub.publish( msg );
    ros::Duration(1).sleep();
  } // while
  ROS_INFO( "%d", msg.data );
  ros::spin();

} // main()

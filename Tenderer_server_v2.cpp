# include <ros/ros.h>
# include <my_demo/bid_price.h>
# include <my_demo/status.h>
# include <std_msgs/String.h>
# include <std_msgs/Int32.h>
# include <stdio.h>
# include <stdlib.h>
# include <time.h>
# include <string.h>

ros::Publisher price_pub ;
int target = 0;
int now_price = 100;
std::string leader = "";
bool client_join = false;
bool client2_join = false;
bool client3_join = false;

void init_price( const ros::TimerEvent& event ) {
  std_msgs::Int32 msg;
  msg.data = now_price;
  price_pub.publish( msg );
} // init_price()

void connect_bidder( const std_msgs::String::ConstPtr& msg ) {
  if ( strcmp( msg->data.c_str(), "bidder1" ) == 0 ) {
    client_join = true;
  } // if()
  else if ( strcmp( msg->data.c_str(), "bidder2" ) == 0 ) {
    client2_join = true;
  } // if()
  else if ( strcmp( msg->data.c_str(), "bidder3" ) == 0 ) {
    client3_join = true;
  } // if()
  else {
    ;
  } // else

} // connect_bidder()

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
    if ( now_price >= target ) {
      ROS_INFO( "Bidding is end, the winner is %s : %d", leader.c_str(),
                 now_price );
    } // if()
    else {
      ROS_INFO( "Now leader is %s : %d", req.leader.c_str(), now_price );
    } // else
    msg.data = now_price;
    price_pub.publish( msg );

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

bool is_ready( my_demo::status::Request &req,
               my_demo::status::Response &res ) {
  ROS_INFO( "someone check" );
  if ( client_join == false || client2_join == false || client3_join == false ) {
    res.start = false;
  } // if()
  else {
    res.start = true;
  } // else

  return true;

} // is_ready()

int main( int argc, char **argv ) {
  std_msgs::Int32 msg;
  ros::init( argc, argv, "Tenderer_server" );
  ros::NodeHandle n;
  ros::ServiceServer service2 = n.advertiseService( "status", is_ready );
  price_pub = n.advertise<std_msgs::Int32>( "price", 1000 );
  ros::Subscriber sub = n.subscribe( "join", 1000, connect_bidder );
  ros::Timer timer = n.createTimer( ros::Duration( 1 ), init_price );
  // ros::ServiceServer service = n.advertiseService( "Bidding", update );
  
  while( client_join == false || client2_join == false || client3_join == false ) {
    ros::spinOnce();
  } // while()

  timer.stop();
  sub.shutdown();

  ros::Duration( 1 ).sleep();
  ros::spinOnce();
  service2.shutdown();
  ros::ServiceServer service = n.advertiseService( "Bidding", update );
  target = 300;
  ROS_INFO( "Now start bidding" );
  ros::spin();

} // main()

# include <ros/ros.h>
# include <my_demo/bid_price.h>
# include <my_demo/status.h>
# include <std_msgs/String.h>
# include <std_msgs/Int32.h>
# include <stdio.h>
# include <stdlib.h>
# include <time.h>
# include <string.h>
# include <thread>

int price = 0;
bool end = false;
bool wait_others = true;
my_demo::bid_price srv;
my_demo::status srv2;
ros::ServiceClient client;
ros::Publisher join;

void get_price( const std_msgs::Int32::ConstPtr& msg ) {
  std_msgs::String name;
  price = msg->data;
  ROS_INFO("price start from %d", price );
  srand( time( 0 ) );
  name.data = "bidder1";
  join.publish( name );
  end = true;
} // get_price

void send_price() { 
  srv.request.price = price;
  srv.request.leader = "chen";
  if ( client.call( srv ) ) {
    if ( srv.response.now_price > price ) {
      wait_others = true;
    } // if()
    else {
      wait_others = true;
    } // esle

  } // if()
  else {
    ROS_ERROR( "Failed to call server" );
  } // else

} // send_price

void scan_price() {
  while( !end ) {
    printf( "please enter price :\n" );
    scanf( "%d", &price );
    send_price();
  } // while()

} // scan_price()

void wait_price( const std_msgs::Int32::ConstPtr& msg ) {

  ROS_INFO("highest price is %d", msg->data );

} // wait_price

int main( int argc, char **argv ) {
  ros::init( argc, argv, "Bidder_client" );
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe( "price", 1000, get_price );
  join = n.advertise<std_msgs::String>( "join", 1000 );
  while( !end ) {
    ros::spinOnce();
  } // while()

  sub.shutdown();
  end = false;
  printf( "get price\n" );

  // client = n.serviceClient<my_demo::bid_price> ( "Bidding" );
  ros::ServiceClient client2 = n.serviceClient<my_demo::status> ( "status" );
  if ( client2.call( srv2 ) ) {
    while( srv2.response.start == false ) {
      if ( client2.call( srv2 ) ) {
        ;
      } // if()
      else {
        ROS_ERROR( "Failed to call status server" );
      } // else
     
      ros::Duration(0.5).sleep();

    } // while()

  } // if()
  else {
    ROS_ERROR( "Failed to call status2 server" );
  } // else

  client = n.serviceClient<my_demo::bid_price> ( "Bidding" );
  sub = n.subscribe( "price", 1000, wait_price );

  std::thread t1( scan_price ); 
  while ( wait_others == true ) {
    ros::spinOnce();
  } // while()
 
  t1.join();

} // main()


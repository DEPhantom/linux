# include <ros/ros.h>
# include <my_demo/bid_price.h>
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
ros::ServiceClient client;

void get_price( const std_msgs::Int32::ConstPtr& msg ) {
  price = msg->data;
  ROS_INFO("price start from %d", price );
  srand( time( 0 ) );
  end = true;
} // get_price

void scan_price() {
  printf( "please enter price :\n" );
  scanf( "%d", &price );
  wait_others = false;
} // scan_price()

void send_price() { 
  srv.request.price = price;
  srv.request.leader = "chen";
  if ( client.call( srv ) ) {
    if ( srv.response.now_price > price ) {
      wait_others = true;
      // price = srv.response.now_price;
      // print new highest price
    } // if()
    else {
      wait_others = true;
    } // esle

  } // if()
  else {
    ROS_ERROR( "Failed to call server" );
  } // else

} // send_price

void wait_price( const std_msgs::Int32::ConstPtr& msg ) {

  ROS_INFO("highest price is %d", msg->data );

} // wait_price

int main( int argc, char **argv ) {
  ros::init( argc, argv, "Bidder_client" );
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe( "price", 1000, get_price );
  while( !end ) {
    ros::spinOnce();
  } // while()

  sub.shutdown();
  end = false;
  printf( "get price\n" );

  client = n.serviceClient<my_demo::bid_price> ( "Bidding" );
  while ( !end ) {
    std::thread t1( scan_price ); 
    if ( wait_others == true ) {
      sub = n.subscribe( "price", 1000, wait_price );
      while ( wait_others == true ) {
        ros::spinOnce();
      } // while()

    } // if()
 
    t1.join();
    sub.shutdown();    
    send_price();

  } // while()

  // srand( time( 0 ) );
  // rand()%5;

} // main()


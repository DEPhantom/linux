#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_demo/FibonacciAction.h>

void done_callback( const actionlib::SimpleClientGoalState& state,
                    const my_demo::FibonacciResultConstPtr& result ) {
  ROS_INFO("Answer: %i", result->sequence.back());
} // done_callback()

void active_callback() {
  ROS_INFO("I don't know what is happen here" );
} // active_callback()

void feedback_callback( const my_demo::FibonacciFeedbackConstPtr& feedback ) {
  ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
} // feedback_callback

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<my_demo::FibonacciAction> ac("fibonacci", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  my_demo::FibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal, &done_callback, &active_callback, &feedback_callback);

  //wait for the action to return
  ROS_INFO("I can do anything, when i waiting.");
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
  ROS_INFO("When i waiting, I can do other thing.");

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}

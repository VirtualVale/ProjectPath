#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <time_experiments/PTSAction.h>
#include <geometry_msgs/PoseStamped.h>

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  actionlib::SimpleActionClient<time_experiments::PTSAction> ac("PTS", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  time_experiments::PTSGoal goal;
  goal.resource_number = 1;
  goal.goal.pose.position.x = (*msg).pose.position.x;
  goal.goal.pose.position.y = (*msg).pose.position.y;
  goal.start_time.data = (*msg).header.stamp;

  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_PTS");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("goaler", 1000, goalCallback);
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<time_experiments::PTSAction> ac("PTS", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  time_experiments::PTSGoal goal;
  goal.resource_number = 1;
  goal.goal.pose.position.x = 0;
  goal.goal.pose.position.y = 5.5;
  goal.start_time.data = ros::Time(100);

  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  goal.resource_number = 1;
  goal.goal.pose.position.x = 6;
  goal.goal.pose.position.y = 5.5;
  goal.start_time.data = ros::Time(115);

  ac.sendGoal(goal);

  //wait for the action to return
  finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  goal.resource_number = 2;
  goal.goal.pose.position.x = 6;
  goal.goal.pose.position.y = 5.5;
  goal.start_time.data = ros::Time(110);

  ac.sendGoal(goal);

  //wait for the action to return
  finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  goal.resource_number = 3;
  goal.goal.pose.position.x = 5.5;
  goal.goal.pose.position.y = 3.5;
  goal.start_time.data = ros::Time(120);

  ac.sendGoal(goal);

  //wait for the action to return
  finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  goal.resource_number = 1;
  goal.goal.pose.position.x = 0;
  goal.goal.pose.position.y = 0;
  goal.start_time.data = ros::Time(130);

  ac.sendGoal(goal);

  //wait for the action to return
  finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  ros::spin();
    

  //exit
  return 0;
}

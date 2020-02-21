#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct goal_type {
  float x_pos;
  float y_pos;
  float orient;
  float time;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "paper_2");

  goal_type list_goals[5];
  list_goals[0] = {1.7, 4, 100, 29};
  list_goals[1] = {5.5, 3, 0, 0};
  list_goals[2] = {3, 0, 100, 0};
  list_goals[3] = {0, 4, 0, 0};
  list_goals[4] = {0, 4, 0, 0};

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("tb3_2/move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose.orientation.w = 1;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  ros::Time time_sum;

  for(int i=0; i<=4; i++){
    ros::Time begin = ros::Time::now();
    goal.target_pose.pose.position.x = list_goals[i].x_pos;
    goal.target_pose.pose.position.y = list_goals[i].y_pos;
    goal.target_pose.pose.orientation.z = list_goals[i].orient;
    //ROS_INFO("Moving to position %d", i);
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Robot3: Postition %d reached!", i);
    else
      ROS_INFO("The base failed to reach position %d for some reason", i);
    ros::Duration time_diff = ros::Time::now() - begin;
    //ROS_INFO("time diff %f", time_diff.toSec());
    time_sum = time_sum + time_diff;
    ros::Duration(list_goals[i].time).sleep();
  }
  ROS_ERROR("Robot3: driveAround-strategy time_sum %f", time_sum.toSec());

  return 0;
}



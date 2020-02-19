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
  ros::init(argc, argv, "paper_1");

  goal_type list_goals[5];
  list_goals[0] = {1, 1, 1, 2};
  list_goals[1] = {6.5, 0.5, 1, 0};
  list_goals[2] = {6.5, 0.5, 1, 0};
  list_goals[3] = {6.5, 0.5, 1, 0};
  list_goals[4] = {6.5, 0.5, 1, 0};

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("tb3_1/move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  for(int i=0; i<=4; i++){
    goal.target_pose.pose.position.x = list_goals[i].x_pos;
    goal.target_pose.pose.position.y = list_goals[i].y_pos;
    goal.target_pose.pose.orientation.w = list_goals[i].orient;
    ROS_INFO("Moving to position %d", i);
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Postition %d reached!", i);
    else
      ROS_INFO("The base failed to reach position %d for some reason", i);
    ros::Duration(list_goals[i].time).sleep();
  }


  return 0;
}



#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  struct goal
  {
    float x_pose;
    float y_pose;
    float orient;
    float time_to_wait;
  };

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("tb3_0/move_base", true);



  std::vector<goal> goals;
  goals.push_back({1, 1, 1, 0});
  goals.push_back({2, 2, 1, 0});

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal_msg;

  for(std::vector<goal>::iterator it = goals.begin(); it != goals.end(); ++it){
      goal_msg.target_pose.header.frame_id = "map";
      goal_msg.target_pose.header.stamp = ros::Time::now();
      //Moving to the start position
      goal_msg.target_pose.pose.position.x = it->x_pose;
      goal_msg.target_pose.pose.position.y = it->y_pose;
      goal_msg.target_pose.pose.orientation.z = 0.0;
      goal_msg.target_pose.pose.orientation.w = it->orient;

      ROS_INFO("%f, %f, %f", it->x_pose, it->y_pose, it->orient);

      ROS_INFO("moving to the start position");
      ac.sendGoal(goal_msg);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Start reached!");
      else
        ROS_INFO("The base failed to move forward 1 meter for some reason");
        
      }

  return 0;
}



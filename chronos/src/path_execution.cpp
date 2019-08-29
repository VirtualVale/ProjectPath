#include "ros/ros.h"
#include "chronos/visualization.h"
#include "nav_msgs/Path.h"
#include "ros/time.h"
#include <std_msgs/Time.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


std::vector<nav_msgs::Path> plan[3];

void planCallback(const chronos::visualization::ConstPtr& msg)
{
    ROS_INFO("planCallback");
    plan[(*msg).resource_number] = (*msg).resource_plan;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "path_executor");
    ros::NodeHandle n;
    ros::Subscriber plan_sub = n.subscribe("plan", 1000, planCallback);
    ros::Duration d(1.0);

      //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("tb3_1/move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

    while (ros::ok())
    {
        ros::Time time = ros::Time::now();
        for(int i=0; i<3;i++)
        {
            for(int j=0; j<plan[i].size();j++)
            {
                if(time.sec == plan[i][j].poses.front().header.stamp.sec)
                {
                    ROS_INFO("Send resource %i on plan %i at time %i", i, j, time.sec);
                    goal.target_pose.header.frame_id = "map";
                    goal.target_pose.header.stamp = ros::Time::now();
                    //Moving to the start position
                    goal.target_pose.pose.position.x = 2.0;
                    goal.target_pose.pose.position.y = 2.0;
                    goal.target_pose.pose.orientation.z = 0.0;
                    goal.target_pose.pose.orientation.w = 1.0;

                    ROS_INFO("moving to the start position");
                    ac.sendGoal(goal);
                    
                    ac.waitForResult();

                    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                        ROS_INFO("Start reached!");
                    else
                        ROS_INFO("The base failed to move forward 1 meter for some reason");
                    
                    d.sleep();
                }
            }
        }
        ros::spinOnce();
    }
    
    return 0;
}
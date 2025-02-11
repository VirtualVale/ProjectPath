#include "ros/ros.h"
#include "chronos/plan.h"
#include "nav_msgs/Path.h"
#include "ros/time.h"
#include <std_msgs/Time.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


std::vector<nav_msgs::Path> plan[3];

void planCallback(const chronos::plan::ConstPtr& msg)
{
    ROS_INFO("Path_execution received the plan.");
    plan[0] = msg -> plan_1;
    plan[1] = msg -> plan_2;
    plan[2] = msg -> plan_3;
    
    //paths are transferred as one path, but have to be splitted to execute them 
    //for this reason the computed paths manipulate the pose sequence number in the following way
    //if the seq is 0 everything is ok, but if there is a pose with seq 1 this signalizes the end of one path and the start of the next
    //so in the following the path is sliced into multiple parts for execution
    nav_msgs::Path slice;
    for(int i=0; i<3;i++)
        {
            for(int j=0; j<plan[i].size();j++)
            {
                for(int k=0; k<plan[i][j].poses.size(); k++)
                {
                    if(plan[i][j].poses[k].header.seq == 1)
                    {
                        slice.poses.clear();
                        //ROS_ERROR("path is sliced at resource %i, path %i and pose %i", i, j, k);
                        slice.header = plan[i][j].header;
                        slice.poses.insert(slice.poses.begin(), plan[i][j].poses.begin(), plan[i][j].poses.begin()+k+1);
                        //ROS_INFO("old path: %lu, new path: %lu, old plan size: %lu", plan[i][j].poses.size(), slice.poses.size(), plan[i].size());
                        //ROS_INFO("last pose seq %i", slice.poses.back().header.seq);
                        slice.poses.back().header.seq = 0;
                        plan[i].push_back(slice);
                        //ROS_INFO("new plan size: %lu", plan[i].size());
                        plan[i][j].poses.erase(plan[i][j].poses.begin(), plan[i][j].poses.begin()+k+1);
                        slice = plan[i][j];
                        plan[i].push_back(slice);
                        plan[i].erase(plan[i].begin() + j);
                        break;
                    }
                }   
            }
        }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "path_executor");
    ros::NodeHandle n;
    ros::Subscriber plan_sub = n.subscribe("plan", 1000, planCallback);
    ros::Duration d(1.0);
    ROS_INFO("Ready to execute paths.");

      //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("tb3_0/move_base", true);
  MoveBaseClient ac1("tb3_1/move_base", true);
  MoveBaseClient ac2("tb3_2/move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
    //in the next loops the time is refreshed every second and if the start time of a path is the current time the path is executed
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
                    //Moving to the goal position
                    goal.target_pose.pose.position.x = plan[i][j].poses.back().pose.position.x;
                    goal.target_pose.pose.position.y = plan[i][j].poses.back().pose.position.y;
                    goal.target_pose.pose.orientation.z = 0.0;
                    goal.target_pose.pose.orientation.w = 1.0;

                    switch (i)
                    {
                        case 0:
                            ac.sendGoal(goal);
                            break;
                        case 1:
                            ac1.sendGoal(goal);
                            break;
                        case 2:
                            ac2.sendGoal(goal);
                            break;
                        default:
                            ROS_ERROR("switch didnt work!");
                    }
                    
                    d.sleep();
                }
            }
        }
        ros::spinOnce();
    }
    
    return 0;
}
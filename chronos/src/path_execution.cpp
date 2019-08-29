#include "ros/ros.h"
#include "chronos/visualization.h"
#include "nav_msgs/Path.h"
#include "ros/time.h"
#include <std_msgs/Time.h>

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
                }
            }
        }
    }
    
    return 0;
}
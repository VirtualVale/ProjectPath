#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "nav_msgs/Path.h"
#include "ros/time.h"
#include "chronos/path_service.h"
#include "chronos/time_service.h"
#include "chronos/collision_service.h"
#include <chronos/PTSAction.h>

bool occupiedBool(ros::Time time, std::vector<nav_msgs::Path> resource_plan);

class PTSAction
{
protected:

    ros::NodeHandle n;
    actionlib::SimpleActionServer<chronos::PTSAction> as_;
    std::string action_name;
    //message for feedback /result
    chronos::PTSFeedback feedback_;
    chronos::PTSResult result_;

    //one plan to rule them all
    std::vector<std::vector<nav_msgs::Path> > plan[99];

    //clients for the required server
    ros::ServiceClient path_client;
    ros::ServiceClient time_client;
    ros::ServiceClient collision_client;

public:

    PTSAction(std::string name) :
        as_(n, name, boost::bind(&PTSAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
        path_client = n.serviceClient<chronos::path_service>("path_service");
        time_client = n.serviceClient<chronos::time_service>("time_service");
        collision_client = n.serviceClient<chronos::collision_service>("collision_service"); 
    }

    ~PTSAction(void)
    {
    }

    void executeCB(const chronos::PTSGoalConstPtr &goal)
    {
        //RESOURCE
        //Resources 1,2,3 are mapped to 0,1,2 for computation reasons
        int resource = (goal-> resource_number) - 1;
        if(resource < 99)
        {
            ROS_INFO("ERROR: Invalid resource number.");
            return false;
        }

        //START TIME
        ros::Time startTime = goal->start_time.data;
        if(occupiedBool(startTime, plan[resource])){
            ROS_INFO("ERROR: Resource is busy at this start time.")
        }

        //START POSITION DETERMINATION
        double startX;
        double startY; 

        if(plan[resource].size() == 0 || startTime < plan[resource].front()) // no paths registered
        {
            switch (resource)
            {
                case 0:
                startX = 0;
                startY = 0;
                break;
                case 1:
                startX = 0;
                startY = 1;
                break;
                case 2:
                startX = 0;
                startY = 2;
                break;
                default:
                ROS_INFO("Error: Invalid resource number");
            }
        } else {

        }
        
    }
}

bool occupiedBool(ros::Time time, std::vector<nav_msgs::Path> resource_plan)
{
    for(int i=0; i<resource_plan.size(); i++)
    {
        if(startTime >= resource_plan[i].poses.front().header.stamp && startTime <= resource_plan[i].poses.back().header.stamp)
        {
            return true;
        }
    }
    return false;
}




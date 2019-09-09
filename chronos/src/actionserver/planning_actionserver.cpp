#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "nav_msgs/Path.h"
#include "ros/time.h"
#include "chronos/path_service.h"
#include "chronos/time_service.h"
#include "chronos/collision_service.h"
#include <chronos/planningAction.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include "chronos/visualization.h"

class planningAction
{
protected:

    ros::NodeHandle n;
    actionlib::SimpleActionServer<chronos::planningAction> as_;
    std::string action_name_;
    //message for feedback /result
    chronos::planningFeedback feedback_;
    chronos::planningResult result_;

    //one plan to rule them all
    std::vector<nav_msgs::Path>  plan[3];

    //clients for the required server
    ros::ServiceClient path_client;
    ros::ServiceClient time_client;
    ros::ServiceClient collision_client;

    //plan publisher
    ros::Publisher plan_pub;
    

public:

    planningAction(std::string name) :
        as_(n, name, boost::bind(&planningAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
        path_client = n.serviceClient<chronos::path_service>("path_service");
        time_client = n.serviceClient<chronos::time_service>("time_service");
        collision_client = n.serviceClient<chronos::collision_service>("collision_service"); 

        plan_pub = n.advertise<chronos::visualization>("plan", 1000);
    }

    ~planningAction(void)
    {
    }

    //checks wether a resource is occupied at the transferred time or not (checked)
    bool checkOccupancy(int resource_id, ros::Time start_time)
    {
        for(int i=0; i<plan[resource_id].size(); i++)
        {
            if(start_time >= plan[resource_id][i].poses.front().header.stamp && start_time <= plan[resource_id][i].poses.back().header.stamp)
            {
                return true;                
            }
        }
        return false;
    }
    
    //creating the requested job 
    //includes checking the parameter and finding the best fitting resource (checked)
    bool createJob(ros::Time start_time ,geometry_msgs::PoseStamped goal)
    {
        ROS_INFO("Creating Job.");
        //PARAMETERCHECK
        //depends on the provided map
        ROS_INFO("Check the goal.");
            

        ROS_INFO("Check the time or look which resource is available.");
        int travel_time;
        int time_min = 1000000;
        int resource_min = -1;
        nav_msgs::Path path_shortest;
        for(int i=0; i<plan.size(); i++)
        {
            if(!plan[i].empty())
            {
                if(!checkOccupancy(start_time, plan[i]))
                {
                    ROS_INFO("Resource %i free.", i);
                    travel_time = createPath(i, goal, start_time);
                    ROS_INFO("Resource %i needs %i secs to reach the transfered goal.", i, travel_times[i]);
                    if(travel_time < time_min)
                    {
                        time_min = travel_time;
                        resource_min = i;
                        path_shortest = response.path;
                    }
                }
            }
        }

        if(resource_min == -1)
        {
            ROS_INFO("No resource is free!");
            return false;
        }

        //REACTION TO POSSIBLE COLLISIONS
        char answer;
        std::cout << "Should the path be added to the plan of the resource? (y/n)";
        std::cin >> answer;
        if(answer == 'y')
        {
            if(insertPath(resource_id, path_shortest)
            {
                ROS_INFO("Path inserted.");
            } else {
                ROS_INFO("Path insertion failed.");
            }
        }
    }

    //pathcreation gives back the time the resource needs to execute the job (checked)
    int createPath(int resource_id, ros::Time start_time, geometry_msgs::PoseStamped goal)
    {
            
        ROS_INFO("Waiting for PTS to start.");
        ac.waitForServer();
        ROS_INFO("PTS ready, sending goal");
            
        chronos::PTSgoal pts_goal;
        pts_goal.resource_number = resource_id;
        pts_goal.goal = goal;
        pts_goal.start_time.data = start_time;
            
        ac.sendGoal(pts_goal);
        
        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else
            ROS_INFO("Action did not finish before the time out.");

        return ac.getResult();
    }

    bool deletePath(int resource_id, ros::Time start_time)
    {
        int pathID_to_delete = getPathID(resource_id, start_time)
        
        //deletewithErase ?
    }

    bool executeCB(const chronos::planningGoalConstPtr &goal)
    {
   
    }

    bool insertPath(int resource_id, nav_msgs::Path createdPath)
    {
        if(plan.empty())
        {
            plan.push_back(createdPath);
            return true;
        } else {
            std::vector<nav_msgs::Path>::iterator it = plan.begin();
        for(int i=0; i<plan.size(); i++)
        {
        if(plan[i].poses.front().header.stamp > createdPath.poses.front().header.stamp)
            {
                plan.insert(it+i, createdPath);
                ROS_INFO("Path inserted at %i, pointer at %i th path", i, i+1);
                nav_msgs::Path successor = createPath(createdPath.poses.back(), plan[i+1].poses.back(), plan[i+1].poses.front().header.stamp, path_client, time_client);
                plan[i+1] = successor;
                return true;
            }
        }
        plan.push_back(createdPath);
        return true;
        }
    }

    //find the path with the transferred start_time in a resource path (checked)
    int getPathID(int resource_id, ros::Time start_time)
    {
        int diff, diff_min = 100000;
        int path_id;
        for(int i=0; i<plan[resource_id]; i++)
        {
            diff = abs(plan[i].poses.header.stamp.toSec()-start_time);
            if(diff < diff_min)
            {
                diff_min = diff;
                path_id = i;
            }
        }
        return path_id;
    }

    bool updateTimeAtPath(int resource_id, ros::Time old_start_time, ros::Time new_start_time)
    {
        //todo implement function with the time_client
    }

    bool updateStartAtPath(int resource_id, ros::Time start_time, geometry_msgs::PoseStamped new_start)
    {
        //todo implement with just a new path with createPath
    }

    bool updateGoalAtPath(int resource_id, ros::Time start_time, geometry_msgs::PoseStamped new_goal)
    {
        //todo implement with createPath and replace the old path
    }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "planning_actionserver");
    planningAction planning_actionserver("planning_actionserver");
    ros::NodeHandle n;
    while (ros::ok())
    {
        ros::spinOnce();
    }
    
    return 0;
}
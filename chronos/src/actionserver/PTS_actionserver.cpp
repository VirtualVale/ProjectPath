#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "nav_msgs/Path.h"
#include "ros/time.h"
#include "chronos/path_service.h"
#include "chronos/time_service.h"
#include "chronos/collision_service.h"
#include <chronos/PTSAction.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include "chronos/visualization.h"



bool occupiedBool(ros::Time time, std::vector<nav_msgs::Path> resource_plan);
nav_msgs::Path createPath( geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal, ros::Time startTime, ros::ServiceClient path_client, ros::ServiceClient time_client);
nav_msgs::Path pathAtTime(ros::Time currentTime, std::vector<nav_msgs::Path> plan);
std::vector<nav_msgs::Path> insertPath(nav_msgs::Path path, std::vector<nav_msgs::Path> plan, ros::ServiceClient path_client, ros::ServiceClient time_client);

class PTSAction
{
protected:

    ros::NodeHandle n;
    actionlib::SimpleActionServer<chronos::PTSAction> as_;
    std::string action_name_;
    //message for feedback /result
    chronos::PTSFeedback feedback_;
    chronos::PTSResult result_;

    //one plan to rule them all
    std::vector<nav_msgs::Path>  plan[99];

    //clients for the required server
    ros::ServiceClient path_client;
    ros::ServiceClient time_client;
    ros::ServiceClient collision_client;

    //plan publisher
    ros::Publisher plan_pub;
    

public:

    PTSAction(std::string name) :
        as_(n, name, boost::bind(&PTSAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
        path_client = n.serviceClient<chronos::path_service>("path_service");
        time_client = n.serviceClient<chronos::time_service>("time_service");
        collision_client = n.serviceClient<chronos::collision_service>("collision_service"); 

        plan_pub = n.advertise<chronos::visualization>("plan", 1000);
    }

    ~PTSAction(void)
    {
    }

    bool executeCB(const chronos::PTSGoalConstPtr &goal)
    {
        //RESOURCE
        //Resources 1,2,3 are mapped to 0,1,2 for computation reasons
        int resource = (goal-> resource_number) - 1;
        if(resource > 99 || resource < 0)
        {
            ROS_INFO("ERROR: Invalid resource number.");
            return false;
        }

        //START TIME
        ros::Time startTime = goal->start_time.data;
        if(occupiedBool(startTime, plan[resource])){
            ROS_INFO("ERROR: Resource is busy at this start time.");
            as_.setAborted();
            return false;
        }

        //START POSITION DETERMINATION
        geometry_msgs::PoseStamped startPose; 

        if(plan[resource].size() == 0 || startTime < plan[resource].front().header.stamp) // no paths registered or path is before first path
        {
            switch (resource)
            {
                case 0:
                startPose.pose.position.x = 0;
                startPose.pose.position.y = 0;
                break;
                case 1:
                startPose.pose.position.x = 0;
                startPose.pose.position.y = 1;
                break;
                case 2:
                startPose.pose.position.x = 0;
                startPose.pose.position.y = 2;
                break;
                default:
                ROS_INFO("Error: Start position for resource number not implemented");
                return false;
            }
        } else {
            nav_msgs::Path lastPath = pathAtTime(startTime, plan[resource]);
            //time is between zero time and last path
            startPose.pose.position.x = lastPath.poses.back().pose.position.x;
            startPose.pose.position.y = lastPath.poses.back().pose.position.y;
        }
        ROS_INFO("PATHCREATION: resources: start position x [%.2lf] y [%.2lf]", startPose.pose.position.x, startPose.pose.position.y);

        //GOAL POSITION
        geometry_msgs::PoseStamped goalPose;
        goalPose.pose.position.x = goal->goal.pose.position.x;
        goalPose.pose.position.y = goal->goal.pose.position.y;
        //TODO check the input

        //PATHCREATION
        nav_msgs::Path createdPath;
        createdPath = createPath(startPose, goalPose, startTime, path_client, time_client);
        
        //COLLISIONCHECKING
        chronos::collision_service csrv;
        csrv.request.superior = createdPath;
        
        //check possible collisions with paths from bigger resource numbers
        for(int i=resource+1; i<99; i++)
        {
            for(int j=0; j<plan[i].size(); j++)
            {
                ROS_INFO("COLLISIONCHECKING: plan_nr [%i], resource [%i]", j, i);
                if(!plan[i].empty())
                {
                    csrv.request.inferior = plan[i][j];
                    if(collision_client.call(csrv))
                    {
                        ROS_INFO("Collisions #: %lu", csrv.response.collisionTimes.data.size());
                    }
                    else
                    {
                        ROS_INFO("ERROR: Collisionchecking failed.");
                    }
                }
            }
        }

        //check possible collision with paths with smaller resource numbers
        for(int i=resource-1; i>=0; i--)
        {
            for(int j=0; j<plan[i].size(); j++)
            {
                ROS_INFO("COLLISIONCHECKING: plan_nr [%i], resource [%i]", j, i);
                if(!plan[i].empty())
                {
                    csrv.request.inferior = plan[i][j];
                    if(collision_client.call(csrv))
                    {
                        ROS_INFO("Collisions #: %lu", csrv.response.collisionTimes.data.size());
                    }
                    else
                    {
                        ROS_INFO("ERROR: Collisionchecking failed.");
                    }
                }
            }
        }

        //REACTION TO POSSIBLE COLLISIONS
        char answer;
        std::cout << "Should the path be added to the plan of the resource? (y/n)";
        std::cin >> answer;
        if(answer == 'y')
        {
            //ADDING PATH TO PLAN
            plan[resource] = insertPath(createdPath, plan[resource], path_client, time_client);
            ROS_INFO("Plan added.");

            chronos::visualization visu;
            visu.resource_number = resource;
            visu.resource_plan = plan[resource];
            plan_pub.publish(visu);

            ROS_INFO("plan:");
            for(int i=0; i<3; i++){
                ROS_INFO("Resource %i:", i);
                for(int j=0; j<plan[i].size(); j++){
                    ROS_INFO("plan %i, start [%.2lf][%.2lf], goal [%.2lf][%.2lf]", j, plan[i][j].poses.front().pose.position.x, plan[i][j].poses.front().pose.position.y, plan[i][j].poses.back().pose.position.x, plan[i][j].poses.back().pose.position.y);
                }
            }

            result_.travel_time = abs(createdPath.poses.front().header.stamp.toSec() - createdPath.poses.back().header.stamp.toSec());
            as_.setSucceeded(result_);
            return true;
        }

        as_.setAborted();
        return true;
    }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "PTS");
    PTSAction PTS("PTS");
    ros::NodeHandle n;
    while (ros::ok())
    {
        ros::spinOnce();
    }
    
    return 0;
}


//calculates if a timestamp lies within a already registered path
bool occupiedBool(ros::Time time, std::vector<nav_msgs::Path> resource_plan)
{
    for(int i=0; i<resource_plan.size(); i++)
    {
        if(time >= resource_plan[i].poses.front().header.stamp && time <= resource_plan[i].poses.back().header.stamp)
        {
            return true;
        }
    }
    return false;
}

//returns the Id of the path the translated timestamp lies within
nav_msgs::Path pathAtTime(ros::Time currentTime, std::vector<nav_msgs::Path> plan)
{
   nav_msgs::Path searchedPath;
   double diff, optDiff = 100.00;
   for(int i=0; i<plan.size(); ++i){
     for(int j=0; j<plan[i].poses.size(); ++j){
       diff = abs(currentTime.toSec() - plan[i].poses[j].header.stamp.toSec());
       if(diff < optDiff){
         optDiff = diff;
         searchedPath = plan[i];
       }
     }
     //ROS_INFO("for-Loop(currentPath) i = %i, optDiff = %lf, diff = %lf, pathID = %i", i, optDiff, diff, pathID);
   }
   return searchedPath;
}

//pathplanning and timestamping, RETURN the plan
nav_msgs::Path createPath( geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal, ros::Time startTime, ros::ServiceClient path_client, ros::ServiceClient time_client)
{
    //path service call
    chronos::path_service psrv;
    psrv.request.start.header.frame_id = "map";
    psrv.request.start.pose.position.x = start.pose.position.x;
    psrv.request.start.pose.position.y = start.pose.position.y;
    psrv.request.start.pose.orientation.w = 1.0;
    psrv.request.goal.header.frame_id = "map";
    psrv.request.goal.pose.position.x = goal.pose.position.x;
    psrv.request.goal.pose.position.y = goal.pose.position.y;
    psrv.request.goal.pose.orientation.w = 1.0;
    if(path_client.call(psrv))
    {
        ROS_INFO("Path size: %lu", psrv.response.path.poses.size());
    } else {
        ROS_INFO("Path creation failed!");
    }

    //time service call
    chronos::time_service tsrv;
    tsrv.request.path = psrv.response.path;
    tsrv.request.startTime = startTime.toSec();
    tsrv.request.average_velocity = 0.5;
    if(time_client.call(tsrv)){
      ROS_INFO("start time: %.2lf, end time: %.2lf", tsrv.response.path_timestamped.poses.front().header.stamp.toSec(), tsrv.response.path_timestamped.poses.back().header.stamp.toSec());
    } else {
      ROS_INFO("Timestamping failed!");
    }    

    return tsrv.response.path_timestamped;
}

std::vector<nav_msgs::Path> insertPath(nav_msgs::Path createdPath, std::vector<nav_msgs::Path> plan, ros::ServiceClient path_client, ros::ServiceClient time_client)
{
        if(plan.empty())
        {
            plan.push_back(createdPath);
            return plan;
        }
        else
        {
            std::vector<nav_msgs::Path>::iterator it = plan.begin();
            for(int i=0; i<plan.size(); i++)
            {
                if(plan[i].poses.front().header.stamp > createdPath.poses.front().header.stamp)
                {
                    plan.insert(it+i, createdPath);
                    ROS_INFO("Path inserted at %i, pointer at %i th path", i, i+1);
                    nav_msgs::Path successor = createPath(createdPath.poses.back(), plan[i+1].poses.back(), plan[i+1].poses.front().header.stamp, path_client, time_client);
                    plan[i+1] = successor;
                    return plan;
                }
            }
            plan.push_back(createdPath);
            //ROS_INFO("ERROR: path insertion found no correct place");
            return plan;
        }
}
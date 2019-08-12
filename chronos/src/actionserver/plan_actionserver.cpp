#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "nav_msgs/Path.h"
#include "ros/time.h"
#include "chronos/path_service.h"
#include "chronos/time_service.h"
#include "chronos/collision_service.h"
#include <chronos/PTSAction.h>
#include <geometry_msgs/Pose.h>

bool occupiedBool(ros::Time time, std::vector<nav_msgs::Path> resource_plan);
nav_msgs::Path createPath( geometry_msgs::Pose start, geometry_msgs::Pose goal, ros::Time startTime, ros::ServiceClient path_client, ros::ServiceClient time_client);


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
        if(resource > 99)
        {
            ROS_INFO("ERROR: Invalid resource number.");
            return false;
        }

        //START TIME
        ros::Time startTime = goal->start_time.data;
        if(occupiedBool(startTime, plan[resource])){
            ROS_INFO("ERROR: Resource is busy at this start time.")
            return false;
        }

        //START POSITION DETERMINATION
        geometry_msgs::Pose start; 

        if(plan[resource].size() == 0 || startTime < plan[resource].front()) // no paths registered
        {
            switch (resource)
            {
                case 0:
                start.position.x = 0;
                start.position.y = 0;
                break;
                case 1:
                start.position.x = 0;
                start.position.y = 1;
                break;
                case 2:
                start.position.x = 0;
                start.position.y = 2;
                break;
                default:
                ROS_INFO("Error: Start position for resource number not implemented");
                return false;
            }
        } else {
            int pathId = pathAtTime(startTime, plan[resource]);
            //time is between zero time and last path
            start.position.x = plan[resource][pathId].poses.back().pose.position.x;
            start.position.y = plan[resource][pathId].poses.back().pose.position.y;
        }

        //GOAL POSITION
        geometry_msgs::Pose goal;
        goal.position.x = goal->goal.pose.position.x;
        goal.position.y = goal->goal.pose.position.y;
        //TODO check the input

        //COLLISIONCHECKING
        //TODO call create Path and look for the chronological order where to insert the created path
        as_.setSucceeded(result_);
    }
}

int int main(int argc, char *argv[])
{
    ros::init(argc, argv, "PTS");
    PTSAction PTS("PTS");
    ros::NodeHandle n;
    ros::spin();
    return 0;
}

//calculates if a timestamp lies within a already registered path
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

//returns the Id of the path the translated timestamp lies within
int pathAtTime(ros::Time currentTime, std::vector<nav_msgs::Path> plan){

   int pathID = 0;
   double diff, optDiff = 100.00;
   for(int i=0; i<plan.size(); ++i){
     for(int j=0; j<plan[i].poses.size(); ++j){
       diff = abs(currentTime.toSec() - plan[i].poses[j].header.stamp.toSec());
       if(diff < optDiff){
         optDiff = diff;
         pathID = i;
       }
     }
     //ROS_INFO("for-Loop(currentPath) i = %i, optDiff = %lf, diff = %lf, pathID = %i", i, optDiff, diff, pathID);
   }
   return pathID;
}

//pathplanning and timestamping, RETURN the plan
nav_msgs::Path createPath( geometry_msgs::Pose start, geometry_msgs::Pose goal, ros::Time startTime, ros::ServiceClient path_client, ros::ServiceClient time_client)
{
    //path service call
    chronos::path_service psrv;
    psrv.request.start.header.frame_id = "map";
    psrv.request.start.pose.position.x = start.position.x;
    psrv.request.start.pose.position.y = start.position.y;
    psrv.request.start.pose.orientation.w = 1.0;
    psrv.request.goal.header.frame_id = "map";
    psrv.request.goal.pose.position.x = goal.position.x;
    psrv.request.goal.pose.position.y = goal.position.y;
    psrv.request.goal.pose.orientation.w = 1.0;
    if(path_client.call(psrv))
    {
        ROS_INFO("Path size: %lu", psrv.response.path.poses.size());
    } else {
        ROS_INFO("Path creation failed!");
        return NULL;
    }

    //time service call
    chronos::time_service tsrv;
    tsrv.request.path = psrv.response.path;
    tsrv.request.path.poses.front().header.stamp = time;
    tsrv.request.average_velocity = 0.5;
    if(time_client.call(tsrv)){
      ROS_INFO("start time: %.2lf, end time: %.2lf", tsrv.response.timesim_path.poses.front().header.stamp.toSec(), tsrv.response.timesim_path.poses.back().header.stamp.toSec());
    } else {
      ROS_INFO("Timestamping failed!");
      return NULL;
    }    

    return tsrv.response.path_timestamped;
}

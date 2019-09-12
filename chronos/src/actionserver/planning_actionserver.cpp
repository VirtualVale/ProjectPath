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
#include "chronos/PTSAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "chronos/plan.h"


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

        plan_pub = n.advertise<chronos::plan>("plan", 1000);
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

        ROS_INFO("Availability Check.");
        int travel_time;
        int time_min = 1000000;
        int resource_min = -1;
        nav_msgs::Path path_shortest, path_created;
        for(int i=0; i<3; i++)
        {
            if(!checkOccupancy(i, start_time))
            {
                path_created = createPath(i, start_time, goal);
                travel_time = path_created.poses.back().header.stamp.toSec()-path_created.poses.front().header.stamp.toSec();
                ROS_INFO("Resource %i needs %i secs to reach the transfered goal.", i, travel_time);
                if(travel_time < time_min)
                {
                    time_min = travel_time;
                    resource_min = i;
                    path_shortest = path_created;
                }
            }
        }
        
        if(resource_min == -1)
        {
            ROS_INFO("No resource is free!");
            return false;
        }
        ROS_INFO("Resource %i has won the race.", resource_min);
        //REACTION TO POSSIBLE COLLISIONS
        char answer;
        std::cout << "Should the path be added to the plan of the resource? (y/n)";
        std::cin >> answer;
        if(answer == 'y')
        {
            if(insertPath(resource_min, path_shortest))
            {
                ROS_INFO("Path inserted.");
            } else {
                ROS_INFO("Path insertion failed.");
            }
        }
        return true;
    }

    //pathcreation gives back the time the resource needs to execute the job (checked)
    nav_msgs::Path createPath(int resource_id, ros::Time start_time, geometry_msgs::PoseStamped goal)
    {
        actionlib::SimpleActionClient<chronos::PTSAction> ac("PTS", true);
        ROS_INFO("Waiting for PTS to start.");
        ac.waitForServer();
        ROS_INFO("PTS ready, sending goal");
            
        chronos::PTSGoal pts_goal;
        pts_goal.resource_number = resource_id+1;
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
        {
            ROS_INFO("Action did not finish before the time out.");
        }
        chronos::PTSResult pts_result = *(ac.getResult());
        return pts_result.path;
    }

    //delete the path at the transfered start time (checked)
    bool deletePath(int resource_id, ros::Time start_time)
    {
        int pathID_to_delete = getPathID(resource_id, start_time);
        if(pathID_to_delete == plan[resource_id].size()-1)
        {
            plan[resource_id].pop_back();
        }else{
            std::vector<nav_msgs::Path>::iterator it = plan[resource_id].begin();
            plan[resource_id].erase(it+pathID_to_delete);
            nav_msgs::Path successor = createPath(resource_id, plan[resource_id][pathID_to_delete+1].poses.front().header.stamp, plan[resource_id][pathID_to_delete+1].poses.back());
            plan[resource_id][pathID_to_delete+1] = successor;
        }
        return true;
    }

    //Callback of the Actionserver, recognizes the task and executes
    bool executeCB(const chronos::planningGoalConstPtr &goal)
    {
        ROS_INFO("Callback Planning_actionserver.");
        //PARAMETERCHECK
        int task = (goal -> task);
        if(task<0 || task>8)
        {
            ROS_FATAL("Invalid task call.");
            as_.setAborted();
            return false;
        }
        ROS_INFO("Task: [%i]", task);

        //Resources 1,2,3 are mapped to 0,1,2 for computation reasons
        int resource = (goal-> resource_number) - 1;
        if(resource>3 || resource<0)
        {
            ROS_FATAL("Invalid resource number.");
            as_.setAborted();
            return false;
        }
        ROS_INFO("resource [%i]", resource);

        ros::Time start_time = goal->start_time.data;
        /*if(checkOccupancy(resource, start_time)){
            ROS_FATAL("ERROR: Resource is busy at this start time.");
            as_.setAborted();
            return false;
        }*/
        ROS_INFO("start time [%.2lf]", start_time.toSec());

        ros::Time new_start_time = goal->new_start_time.data;

        geometry_msgs::PoseStamped goalPose;
        goalPose.pose.position.x = goal->goal.pose.position.x;
        goalPose.pose.position.y = goal->goal.pose.position.y;
        //TODO check the input
        ROS_INFO("goal x [%.2lf] y [%.2lf]", goalPose.pose.position.x, goalPose.pose.position.y);

        geometry_msgs::PoseStamped new_goalPose;
        new_goalPose.pose.position.x = goal->new_goal.pose.position.x;
        new_goalPose.pose.position.y = goal->new_goal.pose.position.y;
        //TODO check the input
        ROS_INFO("new goal x [%.2lf] y [%.2lf]", new_goalPose.pose.position.x, new_goalPose.pose.position.y);

        switch (task)
        {
            case 0:
                if(createJob(start_time, goalPose))
                {
                    as_.setSucceeded();
                    ROS_INFO("Job created");
                } else {
                    as_.setAborted();
                }
                break;
            case 1:
                if(deletePath(resource, start_time))
                {
                    as_.setSucceeded();
                    ROS_INFO("Job deleted");
                } else {
                    as_.setAborted();
                }
                break;
            case 2:
                /* code for condition */
                break;
            case 3:
                {
                    nav_msgs::Path path = createPath(resource, start_time, goalPose);

                    char answer;
                    std::cout << "Should the path be added to the plan of the resource? (y/n)";
                    std::cin >> answer;
                    if(answer == 'y')
                    {
                        if(insertPath(resource, path))
                        {
                            ROS_INFO("Path inserted.");
                            as_.setSucceeded();
                        } else {
                            ROS_INFO("Path insertion failed.");
                            as_.setAborted();
                        }
                    }
                }
                
                break;
            case 4:
                /* code for condition */
                break;
            case 5:
                /* code for condition */
                break;
            case 6:
                /* code for condition */
                break;
            case 7:
                /* code for condition */
                break;
            case 8:
                /* code for condition */
                break;                                                                                                
            default:
                ROS_ERROR("Task unclear.");
        }
        
        chronos::plan plan_overall;
        plan_overall.plan_1 = plan[0];
        plan_overall.plan_2 = plan[1];
        plan_overall.plan_3 = plan[2];
        plan_pub.publish(plan_overall);
        return true;
        
    }

    bool insertPath(int resource_id, nav_msgs::Path createdPath)
    {
        if(plan[resource_id].empty())
        {
            plan[resource_id].push_back(createdPath);
            return true;
        }
        std::vector<nav_msgs::Path>::iterator it = plan[resource_id].begin();
        for(int i=0; i<plan[resource_id].size(); i++)
        {
        if(plan[resource_id][i].poses.front().header.stamp > createdPath.poses.front().header.stamp)
            {
                plan[resource_id].insert(it+i, createdPath);
                ROS_INFO("Path inserted at %i, pointer at %i th path", i, i+1);
                nav_msgs::Path successor = createPath(resource_id, plan[resource_id][i+1].poses.front().header.stamp, plan[resource_id][i+1].poses.back());
                plan[resource_id][i+1] = successor;
                return true;
            }
        }
        plan[resource_id].push_back(createdPath);
        return true;
    }

    //find the path with the transferred start_time in a resource path (checked)
    int getPathID(int resource_id, ros::Time start_time)
    {
        int diff, diff_min = 100000;
        int path_id;
        for(int i=0; i<plan[resource_id].size(); i++)
        {
            diff = abs(plan[resource_id][i].poses.front().header.stamp.toSec()-start_time.toSec());
            if(diff < diff_min)
            {
                diff_min = diff;
                path_id = i;
            }
        }
        return path_id;
    }

    //update makes no sense in the given context
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
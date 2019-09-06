#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "ationlib/client/terminal_state.h"
#include "chronos/createJob_planning_service.h"
#include "chronos/PTSAction.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"



class path_database
{
    private:
        ros::NodeHandle n;
        actionlib::SimpleActionClient<chronos::PTSAction> ac("PTS_client", true);
        std::vector<nav_msgs::Path>  plan[3];


    public:
        path_database();
        virtual ~path_database();

        //checks wether a resource is occupied at the transferred time or not
        bool checkOccupancy(ros::Time time, std::vector<nav_msgs::Path> resource_plan)
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
    
        //creating the requested job 
        //includes checking the parameter and finding the best fitting resource
        bool createJob(geometry_msgs::PoseStamped goal, ros::Time start_time)
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
                if(insertPath(plan[resource_min], path_shortest)
                {
                    ROS_INFO("Path inserted.");
                } else {
                    ROS_INFO("Path insertion failed.");
                }
            }
        }

        //pathcreation gives back the time the resource needs to execute the job
        int createPath(int resource_id, geometry_msgs::PoseStamped goal, ros::Time start_time)
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
            int diff_min = 100000;
            int path_id;
            for(int i=0; i<plan[resource_id]; i++)
            {
                int diff = abs(plan[i].poses.header.stamp.toSec()-start_time);
                if(diff < diff_min)
                {
                    diff_min = diff;
                    path_id = i;
                }
            }
            //deletewithErase ?
        }

        bool updateTimeAtPath()

        bool insertPath(std::vector<nav_msgs::Path> plan, nav_msgs::Path createdPath)
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
};


bool createJob(chronos::createJob_planning_service::Request &req, chronos::createJob_planning_service::Response &res)
{

    return true;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("createJob_planning_service", createJob);
    ROS_INFO("Ready to add a path to the plan.");

    ros::spin();

    return 0;
}
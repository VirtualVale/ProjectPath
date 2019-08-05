#include "ros/ros.h"
#include "time_experiments/pathsim.h"
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>

bool simulatePath(time_experiments::pathsim::Request &req, time_experiments::pathsim::Response &res)
{
    std::vector<geometry_msgs::PoseStamped> plan;

    //costmap
    tf::TransformListener tf(ros::Duration(10));
    costmap_2d::Costmap2DROS costmap ("costmap", tf);
    costmap.start();    

    //global planner - navfn
    navfn::NavfnROS navfn;
    navfn.initialize("my_nafvn_planner", &costmap); 
    navfn.makePlan(req.start, req.goal, plan);

    //response 
    //poses is calculated plan
    res.path.poses = plan;
    res.path.header.frame_id = "map";
    
    ROS_INFO("plan size: %d", (int) res.path.poses.size());

    return true;
}
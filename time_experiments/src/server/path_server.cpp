#include "ros/ros.h"
#include "time_experiments/pathsim.h"
#include "nav_msgs/Path.h"
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>



bool simulatePath(time_experiments::pathsim::Request &req, time_experiments::pathsim::Response &res)
{
    nav_msgs::Path path;

    std::vector<geometry_msgs::PoseStamped> plan;

    tf::TransformListener tf(ros::Duration(10));
    costmap_2d::Costmap2DROS costmap ("costmap", tf);

    costmap.start();    

    navfn::NavfnROS navfn;
    navfn.initialize("my_nafvn_planner", &costmap); 
    navfn.makePlan(req.start, req.goal, plan);

    res.path.poses = plan;
    res.path.header.stamp = ros::Time::now();
    res.path.header.frame_id = "map";
    
    ROS_INFO("plan size: %d", (int) res.path.poses.size());

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathsim_s");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("pathsim", simulatePath);
    ROS_INFO("Ready to simulate a path.");

    ros::spin();

    return 0;
}
/*
bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
*/
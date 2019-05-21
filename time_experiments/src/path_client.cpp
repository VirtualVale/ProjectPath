#include "ros/ros.h"
#include "time_experiments/pathsim.h"
#include "time_experiments/timesim.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathsim_c");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<time_experiments::pathsim>("pathsim");
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("Pathsim", 1000);
    time_experiments::pathsim srv;

    srv.request.start.header.frame_id = "map";
    srv.request.start.header.stamp = ros::Time::now();
    srv.request.start.pose.position.x = -2.0;
    srv.request.start.pose.position.y = 1.0;
    srv.request.start.pose.orientation.w = 1.0;

    srv.request.goal.header.frame_id = "map";
    srv.request.goal.header.stamp = ros::Time::now();
    srv.request.goal.pose.position.x = 6.0;
    srv.request.goal.pose.position.y = -5.0;
    srv.request.goal.pose.orientation.w = 1.0;

    if(client.call(srv))
    {
        ROS_INFO("Path size: %i", srv.response.path.poses.size());
        path_pub.publish(srv.response.path);

        

    } else {
        ROS_INFO("Failed");
        return 1;
    }

    ros::ServiceClient time_client = n.serviceClient<time_experiments::timesim>("timesim");
    time_experiments::timesim tsrv;

    tsrv.request.original_path = srv.response.path;
    tsrv.request.average_velocity = 2.0;

    if(time_client.call(tsrv)){
      ROS_INFO("success");
    } else {
      ROS_INFO("Failed");
      return 1;
    }

    return 0;
}


#include "ros/ros.h"
#include "time_experiments/collisionsim.h"
#include "time_experiments/timesim.h"
#include "time_experiments/pathsim.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_test");
    ros::NodeHandle n;

    //client for collision detection: csrv
    ros::ServiceClient cclient = n.serviceClient<time_experiments::collisionsim>("collisionsim");
    time_experiments::collisionsim csrv;

    //client for timestamping: tsrv
    ros::ServiceClient tclient = n.serviceClient<time_experiments::timesim>("timesim");
    time_experiments::timesim tsrv;

    //Creating one hard-coded path
    nav_msgs::Path stdPath;
    ros::Time startTime (100, 0);

    stdPath.header.stamp = ros::Time::now();
    stdPath.header.frame_id = "map";

    std::vector<geometry_msgs::PoseStamped> plan;
    geometry_msgs::PoseStamped pose;
    for(int i=0; i<=100; i++)
    {
        pose.pose.position.x = 0 + i*0.1;
        pose.header.stamp = startTime + ros::Duration(i);
        pose.header.frame_id = "map";
        plan.push_back(pose);
    }
    stdPath.poses = plan;

    //Creating the first path
    nav_msgs::Path path1 = stdPath;

    //Creating a second path
    nav_msgs::Path path2 = stdPath;
 
     path1.poses[0].header.stamp = startTime;

    tsrv.request.original_path = path1;
    tsrv.request.average_velocity = 0.23;

    
    if(tclient.call(tsrv)){
        path1 = tsrv.response.timesim_path;
        
        ROS_INFO("Path (1) last time: %i", tsrv.response.timesim_path.poses[tsrv.response.timesim_path.poses.size()-1].header.stamp.sec);
    } else {
      ROS_INFO("Timestamping 1 failed!");
      return 1;
    }


    path2.poses[0].header.stamp = startTime;

    tsrv.request.original_path = path2;
    tsrv.request.average_velocity = 0.22;

    if(tclient.call(tsrv)){
        path2 = tsrv.response.timesim_path;
        ROS_INFO("Path (2) last time: %i", tsrv.response.timesim_path.poses[tsrv.response.timesim_path.poses.size()-1].header.stamp.sec);
    } else {
      ROS_INFO("Timestamping 2 failed!");
      return 1;
    }

    csrv.request.superior = path1;
    csrv.request.inferior = path2;

    if(cclient.call(csrv)){
        ROS_INFO("Collision at Time: %i", csrv.response.collision.header.stamp.sec);
        ROS_INFO("Collision at Coordinates: %f, %f", csrv.response.collision.pose.position.x, csrv.response.collision.pose.position.y);
    } else {
        ROS_INFO("Paths do not collide!");
        return 1;
    }

    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path", 1000);
    ros::Publisher path2_pub = n.advertise<nav_msgs::Path>("path2", 1000);
    ros::Publisher collision_pub = n.advertise<geometry_msgs::PoseStamped>("collision", 1000);
    
    while(ros::ok()){
        path_pub.publish(path1);
        path2_pub.publish(path2);
        csrv.response.collision.header.frame_id = "map"; //TODO remove
        collision_pub.publish(csrv.response.collision);
    }
    return 0;
}
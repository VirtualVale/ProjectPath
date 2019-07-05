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

    /*
    PARAMETERS
    [1] Length of path
    [2] x distance per position - path1
    [3] y distance per position - path1
    [4] x distance per position - path2
    [5] y distance per position - path2
    [6] average velocity - before decimal point - path1
    [7] average velocity - after decimal point - path1
    [8] average velocity - before decimal point - path2
    [9] average velocity - after decimal point - path2
    */

    //PATHCREATION 1
    nav_msgs::Path path1;
    path1.header.stamp.sec = 100;
    path1.header.frame_id = "map";

    geometry_msgs::PoseStamped pose_variable;
    for(int i = 0; i < atoi(argv[1]); ++i){

        pose_variable.header.seq = i;
        pose_variable.header.stamp.sec = 100;
        pose_variable.header.frame_id = "map";

        pose_variable.pose.position.x = i*atoi(argv[2]);
        pose_variable.pose.position.y = i*atoi(argv[3]);
        pose_variable.pose.orientation.w = 1;
        path1.poses.push_back(pose_variable);

        ROS_INFO("Path1: Position [%i] at x: [%.2f] added.", i, pose_variable.pose.position.x);
    }

    //PATHCREATION 2
    nav_msgs::Path path2;
    path2.header.stamp.sec = 100;
    path2.header.frame_id = "map";

    for(int i = 2; i < atoi(argv[1]); ++i){

        pose_variable.header.seq = i;
        pose_variable.header.stamp.sec = 100;
        pose_variable.header.frame_id = "map";

        pose_variable.pose.position.x = i*atoi(argv[4]);
        pose_variable.pose.position.y = i*atoi(argv[5]);
        pose_variable.pose.orientation.w = 1;
        path2.poses.push_back(pose_variable);

        ROS_INFO("Path2: Position [%i] at x: [%.2f] added.", i, pose_variable.pose.position.x);
    }

    //TIMESTAMPING 1
    tsrv.request.original_path = path1;
    tsrv.request.average_velocity = atoi(argv[6]) + (0.1*atoi(argv[7]));

    if(tclient.call(tsrv)){
        path1 = tsrv.response.timesim_path;
        
        ROS_INFO("Path (1) last time: %i", tsrv.response.timesim_path.poses[tsrv.response.timesim_path.poses.size()-1].header.stamp.sec);
    } else {
      ROS_INFO("Timestamping 1 failed!");
      return 1;
    }

    //TIMESTAMPING 2
    tsrv.request.original_path = path2;
    tsrv.request.average_velocity = atoi(argv[8]) + (0.1*atoi(argv[9]));

    if(tclient.call(tsrv)){
        path2 = tsrv.response.timesim_path;
        ROS_INFO("Path (2) last time: %i", tsrv.response.timesim_path.poses[tsrv.response.timesim_path.poses.size()-1].header.stamp.sec);
    } else {
      ROS_INFO("Timestamping 2 failed!");
      return 1;
    }

    //COLLISIONCHECKING
    csrv.request.superior = path1;
    csrv.request.inferior = path2;

    if(cclient.call(csrv)){
        if(csrv.response.collision.header.seq == -1){
            ROS_INFO("No Collision");
        } else {
            ROS_INFO("Collision at Time: %i", csrv.response.collision.header.stamp.sec);
            ROS_INFO("Collision at Coordinates: %f, %f", csrv.response.collision.pose.position.x, csrv.response.collision.pose.position.y);
        }

    } else {
        ROS_INFO("Collisioncheck failed!");
        //return 1;
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
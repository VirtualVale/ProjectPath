#include "ros/ros.h"
#include "chronos/collision_service.h"
#include "chronos/time_service.h"
#include "geometry_msgs/PoseStamped.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_test");
    ros::NodeHandle n;

    //client for collision detection: csrv
    ros::ServiceClient cclient = n.serviceClient<chronos::collision_service>("collision_service");
    chronos::collision_service csrv;

    //client for timestamping: tsrv
    ros::ServiceClient tclient = n.serviceClient<chronos::time_service>("time_service");
    chronos::time_service tsrv;

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
    if(argc != 10 || !isdigit(*argv[1]) || !isdigit(*argv[2]) || !isdigit(*argv[3]) || !isdigit(*argv[4]) || !isdigit(*argv[5]) || !isdigit(*argv[6]) || !isdigit(*argv[7]) || !isdigit(*argv[8]) || !isdigit(*argv[9])){
        ROS_INFO("CHECK THE PARAMETERS\n[1] Length of path\n[2] x distance per position - path1\n[3] y distance per position - path1\n[4] x distance per position - path2\n[5] y distance per position - path2\n[6] average velocity - before decimal point - path1\n[7] average velocity - after decimal point - path1\n[8] average velocity - before decimal point - path2\n[9] average velocity - after decimal point - path2");
        return 1;
    }
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

    for(int i = 0; i < atoi(argv[1]); ++i){

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
    tsrv.request.path = path1;
    tsrv.request.average_velocity = atoi(argv[6]) + (0.1*atoi(argv[7]));
    tsrv.request.startTime = 100;

    if(tclient.call(tsrv)){
        path1 = tsrv.response.path_timestamped;
        
        ROS_INFO("Path (1) last time: %i", tsrv.response.path_timestamped.poses[tsrv.response.path_timestamped.poses.size()-1].header.stamp.sec);
    } else {
      ROS_INFO("Timestamping 1 failed!");
      return 1;
    }

    //TIMESTAMPING 2
    tsrv.request.path = path2;
    tsrv.request.average_velocity = atoi(argv[8]) + (0.1*atoi(argv[9]));
    tsrv.request.startTime = 100;

    if(tclient.call(tsrv)){
        path2 = tsrv.response.path_timestamped;
        ROS_INFO("Path (2) last time: %i", tsrv.response.path_timestamped.poses[tsrv.response.path_timestamped.poses.size()-1].header.stamp.sec);
    } else {
      ROS_INFO("Timestamping 2 failed!");
      return 1;
    }

    //COLLISIONCHECKING
    csrv.request.superior = path1;
    csrv.request.inferior = path2;

    if(cclient.call(csrv)){
        ROS_INFO("firstCollision at time: %lf", csrv.response.firstCollision.header.stamp);
    } else {
        ROS_INFO("Collisioncheck failed!");
    }

    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path", 1000);
    ros::Publisher path2_pub = n.advertise<nav_msgs::Path>("path2", 1000);
    //ros::Publisher collision_pub = n.advertise<geometry_msgs::PoseStamped>("collision", 1000);
    
    while(ros::ok()){
        path_pub.publish(path1);
        path2_pub.publish(path2);
        //collision_pub.publish(csrv.response.collision);
    }
    return 0;
}
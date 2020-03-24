#include "ros/ros.h"
#include "path_planning_system/time_service.h"
#include "path_planning_system/path_service.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <stdlib.h>

geometry_msgs::PoseWithCovarianceStamped current_pose;

void poseCallback(const geometry_msgs::PoseWithCovarianceStampedPtr& msg)
{
    ROS_INFO("amcl_pose: [x] %lf.2 [y] %lf.2", msg->pose.pose.position.x, msg->pose.pose.position.y);
    current_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "time_test");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<path_planning_system::time_service>("time_service");
    ros::Subscriber pose_sub = n.subscribe("/tb3_0/amcl_pose", 1000, poseCallback);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path_of_time_test", 1000);
    path_planning_system::time_service tsrv;
    path_planning_system::path_service psrv;

    psrv.request.start.pose = current_pose.pose.pose;
    psrv.request.start.header.frame_id = "map";

    psrv.request.goal.header.frame_id = "map";
    psrv.request.goal.pose.position.x = rand() % 10;
    psrv.request.goal.pose.position.y = rand() % 10;
    psrv.request.goal.pose.orientation.w = 1.0;

    if(client.call(psrv))
    {
        ROS_INFO("Path size: %lu\nPath published on the '/path_of_time_test' topic.\n", psrv.response.path.poses.size());

    } else {
        ROS_INFO("Failed");
        return 1;
    }
    while(ros::ok()){
        ros::spinOnce();
        path_pub.publish(psrv.response.path);
    }
    /*
    //input check
    if(argc != 5 || isdigit(atoi(argv[1])) || isdigit(atoi(argv[2])) || isdigit(atoi(argv[3])) || isdigit(atoi(argv[4]))){
        ROS_INFO("Input Parameters are wrong!\nstart position:\nx\ny\ngoal position:\nx\ny");
        return 1;
    }

    //server call
    tsrv.request.start.header.frame_id = "map";
    tsrv.request.start.pose.position.x = atoi(argv[1]);
    tsrv.request.start.pose.position.y = atoi(argv[2]);
    tsrv.request.start.pose.orientation.w = 1.0;

    tsrv.request.goal.header.frame_id = "map";
    tsrv.request.goal.pose.position.x = atoi(argv[3]);
    tsrv.request.goal.pose.position.y = atoi(argv[4]);
    tsrv.request.goal.pose.orientation.w = 1.0;

    if(client.call(tsrv))
    {
        ROS_INFO("Path size: %lu\nPath published on the '/time_test' topic.\n", tsrv.response.path.poses.size());

    } else {
        ROS_INFO("Failed");
        return 1;
    }
    

    while(ros::ok()){
        //path_pub.publish(tsrv.response.path);
    }
    */
    
    return 0;    

    
}
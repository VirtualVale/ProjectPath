#include "ros/ros.h"
#include "chronos/time_service.h"
#include "nav_msgs/Path.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "time_test");
    ros::NodeHandle n;
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("time_test", 1000);
    ros::ServiceClient time_client = n.serviceClient<chronos::time_service>("time_service");
    chronos::time_service tsrv;

    /*
      Parameter [1] Number of positions
                [2] x range between every position
                [3] y range between every position
                [4] average velocity - Vorkomma
                [5] average velocity - nachkomma
                [6] start Time
    */
    //Verification of the parameters
    if(argc != 7 || !isdigit(*argv[1]) || !isdigit(*argv[2]) || !isdigit(*argv[3]) || !isdigit(*argv[4]) || !isdigit(*argv[5]) || !isdigit(*argv[6])){
        
        ROS_INFO("CHECK THE PARAMETERS\n[1] Number of positions\n[2] x range between every position\n[3] y range between every position\n[4] average velocity - before decimal point\n[5] average velocity - after decimal point\n[6] start time");
        return 1;
    }

    //setting up an auto constructed path
    nav_msgs::Path path;
    path.header.frame_id = "map";

    geometry_msgs::PoseStamped pose_variable;

    for(int i = 0; i < atoi(argv[1]); ++i){

        pose_variable.header.seq = i;
        pose_variable.header.stamp.sec = 100;
        pose_variable.header.frame_id = "map";

        pose_variable.pose.position.x = i*atoi(argv[2]);
        pose_variable.pose.position.y = i*atoi(argv[3]);
        pose_variable.pose.orientation.w = 1;
        path.poses.push_back(pose_variable);
    }

    tsrv.request.path = path;
    tsrv.request.average_velocity = atoi(argv[4]) + (0.1*atoi(argv[5]));
    tsrv.request.startTime = atoi(argv[6]);

    if(time_client.call(tsrv)){
      ROS_INFO("Start time: %.2f\nEnd time: %.2f\nAverage traveltime: %.2f", tsrv.response.path_timestamped.poses.front().header.stamp.toSec(), tsrv.response.path_timestamped.poses.back().header.stamp.toSec(),(tsrv.response.path_timestamped.poses.back().header.stamp.toSec() - tsrv.response.path_timestamped.poses.front().header.stamp.toSec()) / tsrv.response.path_timestamped.poses.size()); 
      ROS_INFO("Timestamped path is published on the time_service topic.");
    } else {
      ROS_INFO("Failed");
      return 1;
    }

    while(ros::ok()){
        path_pub.publish(tsrv.response.path_timestamped);
    }
    return 0;    
}    
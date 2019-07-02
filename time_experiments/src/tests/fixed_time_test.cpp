#include "ros/ros.h"
#include "time_experiments/pathsim.h"
#include "time_experiments/timesim.h"
#include "nav_msgs/Path.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "timeserver_test");
    ros::NodeHandle n;
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("Timesim", 1000);

    /*
      Parameter [1] Number of positions
                [2] x range between every position
                [3] y range between every position
                [4] average velocity
    */

    //Verification of the parameters
    if(isdigit(*argv[1]) || isdigit(atoi(argv[2])) || isdigit(atoi(argv[3])) || isspace(atoi(argv[4]))){
        ROS_INFO("CHECK THE PARAMETERS\n[1] Number of positions\n[2] x range between every position\n[3] y range between every position\n[4] average velocity");
        return 1;
    }

    //Implementation of a fix path constructed with the stated parameters
    nav_msgs::Path path_fix;
    path_fix.header.stamp.sec = 100;
    path_fix.header.frame_id = "map";

    geometry_msgs::PoseStamped pose_variable;

    for(int i = 0; i < atoi(argv[1]); ++i){

        pose_variable.header.seq = i;
        pose_variable.header.stamp.sec = 100;
        pose_variable.header.frame_id = "map";

        pose_variable.pose.position.x = i*atoi(argv[2]);
        pose_variable.pose.position.y = i*atoi(argv[3]);
        pose_variable.pose.orientation.w = 1;
        path_fix.poses.push_back(pose_variable);

        ROS_INFO("Position [%i] at x: [%.2f] added.", i, pose_variable.pose.position.x);
    }

    ROS_INFO("Path size: %lu", path_fix.poses.size());

    ros::ServiceClient time_client = n.serviceClient<time_experiments::timesim>("timesim");
    time_experiments::timesim tsrv;

    tsrv.request.original_path = path_fix;
    tsrv.request.average_velocity = atof(argv[4]);

    if(time_client.call(tsrv)){
      ROS_INFO("success\n       Timestamped path is published on the Timesim topic.");
    } else {
      ROS_INFO("Failed");
      return 1;
    }

    while(ros::ok()){
        path_pub.publish(tsrv.response.timesim_path);
    }
    return 0;
}
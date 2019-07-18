#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "ros/time.h"
#include "time_experiments/pathsim.h"
#include "time_experiments/timesim.h"

std::vector<nav_msgs::Path> path_2_vector(std::vector<nav_msgs::Path> vector, float x1, float y1, float x2, float y2, ros::Time time, ros::ServiceClient path_client, ros::ServiceClient time_client);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "visualizer_test");
    ros::NodeHandle n;

    //Publisher for time and the three resources
    ros::Publisher time_pub = n.advertise<ros::Time>("timer", 1000);

    ros::Publisher r1_plan_pub = n.advertise<std_msgs::vector<nav_msgs::Path>>("r1_plan", 1000);
    ros::Publisher r2_plan_pub = n.advertise<std_msgs::vector<nav_msgs::Path>>("r2_plan", 1000);
    ros::Publisher r3_plan_pub = n.advertise<std_msgs::vector<nav_msgs::Path>>("r3_plan", 1000);

    //data generation
    ros::Time snapshot(110);

    //vector(path) for each robot
    std::vector<nav_msgs::Path> r1_plan;
    std::vector<nav_msgs::Path> r2_plan;
    std::vector<nav_msgs::Path> r3_plan;

    //clients for the pathplanning and timestamping
    ros::ServiceClient path_client = n.serviceClient<time_experiments::pathsim>("pathsim");
    ros::ServiceClient time_client = n.serviceClient<time_experiments::timesim>("timesim");

    //plan for r1
    r1_plan = path_2_vector(r1_plan, 0, 0, 0, 5.5, ros::Time(100.00), path_client, time_client);
    r1_plan = path_2_vector(r1_plan, 0, 5.5, 6, 5.5, ros::Time(110.00), path_client, time_client);
    r1_plan = path_2_vector(r1_plan, 6, 5.5, 0, 0, ros::Time(120.00), path_client, time_client);
    ROS_INFO("r1_plan: %lu", r1_plan.size()); //for debugging

    //plan for r2
    r2_plan = path_2_vector(r2_plan, 0, 1, 3.5, 3.5, ros::Time(110.00), path_client, time_client);
    r2_plan = path_2_vector(r2_plan, 3.5, 3.5, 5.5, 3.5, ros::Time(120.00), path_client, time_client);
    r2_plan = path_2_vector(r2_plan, 5.5, 3.5, 0, 1, ros::Time(130.00), path_client, time_client);

    //plan for r3
    r3_plan = path_2_vector(r3_plan, 0, 2, 5.5, 3.5, ros::Time(120.00), path_client, time_client);
    r3_plan = path_2_vector(r3_plan, 5.5, 3.5, 0, 2, ros::Time(130.00), path_client, time_client);

    while(ros::ok()){
        time_pub.publish(snapshot);
        r1_plan_pub(r1_plan);
        r2_plan_pub(r2_plan);
        r3_plan_pub(r3_plan);
        ros::spinOnce();
    }

    return 0;
}

//function for pathplanning and timestamping service call, gives back the updated vector
std::vector<nav_msgs::Path> path_2_vector(std::vector<nav_msgs::Path> vector, float x1, float y1, float x2, float y2, ros::Time time, ros::ServiceClient path_client, ros::ServiceClient time_client){

    //pathplanning call with parameters
    time_experiments::pathsim psrv;
    psrv.request.start.header.frame_id = "map";
    psrv.request.start.header.stamp = time;
    psrv.request.start.pose.position.x = x1;
    psrv.request.start.pose.position.y = y1;
    psrv.request.start.pose.orientation.w = 1.0;
    psrv.request.goal.header.frame_id = "map";
    //psrv.request.goal.header.stamp = ros::Time::now();
    psrv.request.goal.pose.position.x = x2;
    psrv.request.goal.pose.position.y = y2;
    psrv.request.goal.pose.orientation.w = 1.0;
    if(path_client.call(psrv))
    {
        ROS_INFO("Path size: %lu", psrv.response.path.poses.size());
    } else {
        ROS_INFO("Failed");
    }

    //timestamping
    time_experiments::timesim tsrv;
    tsrv.request.original_path = psrv.response.path;
    tsrv.request.original_path.poses.front().header.stamp = time;
    tsrv.request.average_velocity = 0.5;
    if(time_client.call(tsrv)){
      ROS_INFO("start time: %.2lf, end time: %.2lf", tsrv.response.timesim_path.poses.front().header.stamp.toSec(), tsrv.response.timesim_path.poses.back().header.stamp.toSec());
    } else {
      ROS_INFO("Failed");
    }

    vector.push_back(tsrv.response.timesim_path);
    ROS_INFO("vector size: %lu", vector.size());
    return vector;
}
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "time_experiments/pathsim.h"
#include "time_experiments/timesim.h"

std::vector<nav_msgs::Path> path_2_vector(std::vector<nav_msgs::Path> vector, geometry_msgs::PoseStamped x1, geometry_msgs::PoseStamped y1, geometry_msgs::PoseStamped x2, geometry_msgs::PoseStamped y2, ros::Time time, ros::ServiceClient path_client, ros::ServiceClient time_client);
int time_2_poseID(ros::Time snapshot, std::vector<nav_msgs::Path> vector);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "process_example");
    ros::NodeHandle n;

    //time for the snapshot
    ros::Time snapshot(100);

    //Publisher for the first robot
    ros::Publisher r1_pose_pub = n.advertise<nav_msgs::Path>("r1_pose", 1000);
    ros::Publisher r1_path_f_pub = n.advertise<nav_msgs::Path>("r1_path_f", 1000);
    ros::Publisher r1_path_t_pub = n.advertise<nav_msgs::Path>("r1_path_t", 1000);

    //Publisher for the second robot
    ros::Publisher r2_pose_pub = n.advertise<nav_msgs::Path>("r2_pose", 1000);
    ros::Publisher r2_path_f_pub = n.advertise<nav_msgs::Path>("r2_path_f", 1000);
    ros::Publisher r2_path_t_pub = n.advertise<nav_msgs::Path>("r2_path_t", 1000);

    //Publisher for the third robot
    ros::Publisher r3_pose_pub = n.advertise<nav_msgs::Path>("r3_pose", 1000);
    ros::Publisher r3_path_f_pub = n.advertise<nav_msgs::Path>("r3_path_f", 1000);
    ros::Publisher r3_path_t_pub = n.advertise<nav_msgs::Path>("r3_path_t", 1000);

    //vector(path) for each robot
    std::vector<nav_msgs::Path> plan_r1;
    std::vector<nav_msgs::Path> plan_r2;
    std::vector<nav_msgs::Path> plan_r3;

    //clients for the pathplanning and timestamping
    ros::ServiceClient path_client = n.serviceClient<time_experiments::pathsim>("pathsim");
    ros::ServiceClient time_client = n.serviceClient<time_experiments::timesim>("timesim");

    //plan for r1
    plan_r1 = path_2_vector(plan_r1, 0, 0, 0, 5.5, 100.00, path_client, time_client);
    plan_r1 = path_2_vector(plan_r1, 0, 5.5, 6, 5.5, 110.00, path_client, time_client);
    plan_r1 = path_2_vector(plan_r1, 6, 5.5, 0, 0, 120.00, path_client, time_client);

    ROS_INFO("plan_r1: %i", plan_r1.size()); //for debugging

    //go through at first all paths/poses included in the vector
    while(ros::ok()){
        
        geometry_msgs::PoseStamped currentPose = plan_r1. time_2_poseID(snapshot ,plan_r1);


    }

    return 0;
}

//function for pathplanning and timestamping service call, gives back the updated vector
std::vector<nav_msgs::Path> path_2_vector(std::vector<nav_msgs::Path> vector, geometry_msgs::PoseStamped x1, geometry_msgs::PoseStamped y1, geometry_msgs::PoseStamped x2, geometry_msgs::PoseStamped y2, ros::Time time, ros::ServiceClient path_client, ros::ServiceClient time_client){

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
        ROS_INFO("Path size: %lu", srv.response.path.poses.size());
    } else {
        ROS_INFO("Failed");
        return null;
    }

    //timestamping
    tsrv.request.original_path = psrv.response.path;
    tsrv.request.average_velocity = 0.5;
    if(time_client.call(tsrv)){
      ROS_INFO("success\n       Timestamped path is published on the Timesim topic.");
    } else {
      ROS_INFO("Failed");
      return null;
    }

    vector.push_back(tsrv.response.timesim_path);
    ROS_INFO("vector size: %i", vector.size());
    return vector;
}

//search for the position of a pose element at a certain time
int time_2_poseID(ros::Time snapshot, std::vector<nav_msgs::Path> vector){

    for(int i = 0; i<=vector.size(); i++){
        for(int j = 0; j<=vector[i].poses.size(); j++){
            if(vector[i].poses[j].header.stamp > snapshot){

                ROS_INFO("currentPosition %f %f at time %f", vector[i].poses[j-1].pose.position.x, vector[i].poses[j-1].pose.position.y, vector[i].poses[j-1].header.stamp);
                return j-1;

            }
        }
    }

    return vector.back().poses.size() - 1;

}

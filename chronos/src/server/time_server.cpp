#include "ros/ros.h"
#include "chronos/time_service.h"

bool stampTime(chronos::time_service::Request &req, chronos::time_service::Response &res){

    //deny a negative velocity as driving backwards is not interesting in this phase
    if(req.average_velocity <= 0){
        ROS_INFO("Velocities <= 0 are not valid!");
        return 1;
    }
    
    req.path.poses.front().header.stamp = ros::Time(req.startTime);
    for(int i=1; i<req.path.poses.size(); i++){
        //calculate distance btw pair of poses
        double euclideanDistance = sqrt(pow((req.path.poses[i].pose.position.x-req.path.poses[i-1].pose.position.x),2)+pow((req.path.poses[i].pose.position.y-req.path.poses[i-1].pose.position.y),2));
        //calculation of the resulting travel time
        double travel_time = euclideanDistance/ req.average_velocity;
        req.path.poses[i].header.stamp = req.path.poses[i-1].header.stamp + ros::Duration(travel_time);
    }
    ROS_INFO("Start time: [%i] Goal time: [%i]", req.path.poses.front().header.stamp.sec, req.path.poses.back().header.stamp.sec);
    res.path_timestamped = req.path;
    return true;
}

int main(int argc, char **argv)
{
    //time_server
    ros::init(argc, argv, "time_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("time_service", stampTime);
    ROS_INFO("Ready to stamp time.");

    ros::spin();
    return 0;
}
#include "ros/ros.h"
#include "time_experiments/timesim.h"

bool simulateTime(time_experiments::timesim::Request &req, time_experiments::timesim::Response &res)
{
    //relations between poses of the path have to be specified 
    bool trigger = false;
    while(ros::ok() && trigger == false){
        if( req.original_path.poses.size() == 0){
        // prevent the case of no path TODO is it really necessary?
        } else {
            for(int i = 0; i < req.original_path.poses.size()-1; i++){
            //first the distance between the current poses
            double euclideanDistance = sqrt(pow((req.original_path.poses[i].pose.position.x-req.original_path.poses[i+1].pose.position.x),2)+pow((req.original_path.poses[i].pose.position.y-req.original_path.poses[i+1].pose.position.y),2));
            //second the time for the distance to travel 
            double travel_time = euclideanDistance/ req.average_velocity; //TODO deny zero as input for average velocity
    
            req.original_path.poses[i+1].header.stamp = req.original_path.poses[i].header.stamp + ros::Duration(travel_time);
                if(i==0 || i==1 /*req.original_path.poses.size()-2*/){
                    ROS_INFO("time: [%i]", req.original_path.poses[i+1].header.stamp.sec);
                    ROS_INFO("distance: [%f]", euclideanDistance);
                }
            }
            trigger = true;
        }
    }
    res.timesim_path = req.original_path;
    return true;
}

int main(int argc, char **argv)
{
    //timesim_s(erver)
    ros::init(argc, argv, "timesim_s");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("timesim", simulateTime);
    ROS_INFO("Ready to simulate time.");

    ros::spin();
    return 0;
}
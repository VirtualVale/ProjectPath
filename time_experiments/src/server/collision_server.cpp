#include "ros/ros.h"
#include "time_experiments/collisionsim.h"

bool simulateCollision(time_experiments::collisionsim::Request &req, time_experiments::collisionsim::Response &res){
    
    if(req.superior.poses.empty() || req.inferior.poses.empty())return false;
    
    for(int i=0; i<req.inferior.poses.size();i++){  
        for(int j=0; j<req.superior.poses.size();j++){

            double euclideanDistance = sqrt(pow((req.inferior.poses[i].pose.position.x-req.superior.poses[j].pose.position.x),2)+pow((req.inferior.poses[i].pose.position.y-req.superior.poses[j].pose.position.y),2));
            
            if(euclideanDistance<0.5){
                ROS_INFO("Collision detected.");
                res.collision.header.stamp = ros::Time::now();
                res.collision.pose = req.inferior.poses[i];
                res.collision.collision = true;
                return true;
            }else{//TODO necessary?
            }
        }
    }

    res.collision.collision = false;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collisionsim_s");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("collisionsim", simulateCollision);
    ROS_INFO("Ready to check for collisions.");

    ros::spin();
    return 0;
}

//TODO functions checkPath_geometric, checkPath_chronologic
#include "ros/ros.h"
#include "time_experiments/collisionsim.h"

bool distance_check(nav_msgs::Path path_sup, nav_msgs::Path path_inf);
bool time_check(geometry_msgs::PoseStamped pose_sup, geometry_msgs::PoseStamped pose_inf);
float euclideanDistance(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b);

//variable to return collisionpose
int pose_collision = -1;

//
bool collision_check(time_experiments::collisionsim::Request &req, time_experiments::collisionsim::Response &res){
//TODO Check input
    pose_collision = -1;

    if(distance_check(req.superior, req.inferior)){
        res.collision = req.inferior.poses[pose_collision];
        return true;
    }
    res.collision.header.seq = pose_collision;
    ROS_INFO("No Collision.");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collisionsim_s");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("collisionsim", collision_check);
    ROS_INFO("Ready to check for collisions.");

    ros::spin();
    return 0;
}

//check if the distance between the single poses of each path is big enough to allow the way for both robots
bool distance_check(nav_msgs::Path path_sup, nav_msgs::Path path_inf){

    for(int i=0; i<path_inf.poses.size();i++){  
        for(int j=0; j<path_sup.poses.size();j++){
            double distance = euclideanDistance(path_inf.poses[i], path_sup.poses[j]);
            if(distance < 0.096){
                if(time_check(path_sup.poses[j],path_inf.poses[i])){
                    pose_collision = i;
                    return true;
                }
            }
        }
    }
    return false;

}

bool time_check(geometry_msgs::PoseStamped pose_sup, geometry_msgs::PoseStamped pose_inf){

    double temporal_distance = fabs(pose_inf.header.stamp.toSec() - pose_sup.header.stamp.toSec());
    ROS_INFO("%f seconds.nsecs between the Poses.", temporal_distance);
    
    if(temporal_distance < 0.627)return true;
    return false;
}

float euclideanDistance(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b){

    float diffX = a.pose.position.x-b.pose.position.x;
    float diffY = a.pose.position.y-b.pose.position.y;
    float dist = sqrt(pow(diffX,2)+pow(diffY,2));
    return dist;
}
/*
int checkPath_geometric(nav_msgs::Path superior, nav_msgs::Path inferior, int PoseID);
bool checkPath_chronologic(int collisionPose, nav_msgs::Path superior, nav_msgs::Path inferior);

//checks collision of two paths in a geometric and chronological manner, returns true if there is a collision
bool simulateCollision(time_experiments::collisionsim::Request &req, time_experiments::collisionsim::Response &res){
    
    bool endOfPath = true;

    if(req.superior.poses.empty() || req.inferior.poses.empty())return false;

    int poseID = 0;

    while(endOfPath){
        poseID = checkPath_geometric(req.superior, req.inferior, poseID);


            if(poseID == -1)return false;
            if(checkPath_chronologic(poseID, req.superior, req.inferior)){
                res.collision.header.stamp = ros::Time::now();
                res.collision.pose = req.inferior.poses[poseID].pose;
                return true; //paths colliding
            }
            if(poseID == req.inferior.poses.size()-1)return false;

        
        ++poseID;
    }
    

    res.collision.header.stamp = ros::Time::now();
    res.collision.pose = req.inferior.poses[poseID].pose;
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

//geometric Check for collisions
int checkPath_geometric(nav_msgs::Path superior, nav_msgs::Path inferior, int poseID)
{
    for(int i=poseID; i<inferior.poses.size();i++){  
        for(int j=0; j<superior.poses.size();j++){

            double euclideanDistance = sqrt(pow((inferior.poses[i].pose.position.x-superior.poses[j].pose.position.x),2)+pow((inferior.poses[i].pose.position.y-superior.poses[j].pose.position.y),2));

            if(euclideanDistance<0.069){ //Distance is Radius of a burger
                ROS_INFO("Geometrical collision detected.");
                ROS_INFO("Pose %i on Path1.", j);
                ROS_INFO("Pose %i on Path2.", i);
                return i;
                //returns the pose where collision is possible (inferior path)
            }
        }
    }
    return -1;
    //no collision
}

//chronological comparison of poses 
bool checkPath_chronologic(int collisionPose, nav_msgs::Path superior, nav_msgs::Path inferior){
    
    double timeBtwPoses = fabs(inferior.poses[collisionPose].header.stamp.toSec() - superior.poses[collisionPose].header.stamp.toSec()); 
    
    ROS_INFO("Path 1 time: %f", inferior.poses[collisionPose].header.stamp.toSec());
    ROS_INFO("Path 2 time: %f", superior.poses[collisionPose].header.stamp.toSec());
    ROS_INFO("timeBtwPoses: %f", timeBtwPoses);

    if(timeBtwPoses > 0.627){
        ROS_INFO("Duration btw Poses is enough.Go!");
        ROS_INFO("%f seconds between the Poses.", timeBtwPoses);
        return false;
    } else {
        ROS_INFO("Chronological collision detected.");
        ROS_INFO("Time %f on Path1.", superior.poses[collisionPose].header.stamp.toSec());
        ROS_INFO("Time %f on Path2.", inferior.poses[collisionPose].header.stamp.toSec());
        ROS_INFO("%f seconds between the Poses.", timeBtwPoses);
        return true;
    }
    
}
*/
#include "ros/ros.h"
#include "time_experiments/collisionsim.h"

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
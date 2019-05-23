#include "ros/ros.h"
#include "time_experiments/collisionsim.h"

int checkPath_geometric(nav_msgs::Path superior, nav_msgs::Path inferior);
bool checkPath_chronologic(int collisionPose, nav_msgs::Path superior, nav_msgs::Path inferior);

//checks collision of two paths in a geometric and chronological manner, returns true if there is a collision
bool simulateCollision(time_experiments::collisionsim::Request &req, time_experiments::collisionsim::Response &res){
    
    if(req.superior.poses.empty() || req.inferior.poses.empty())return false;

    int poseID = checkPath_geometric(req.superior, req.inferior);
    if(poseID >= 0){
        if(checkPath_chronologic(poseID, req.superior, req.inferior))return false; //paths colliding but the duration between the motions is enough
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
int checkPath_geometric(nav_msgs::Path superior, nav_msgs::Path inferior)
{
    for(int i=0; i<inferior.poses.size();i++){  
        for(int j=0; j<superior.poses.size();j++){

            double euclideanDistance = sqrt(pow((inferior.poses[i].pose.position.x-superior.poses[j].pose.position.x),2)+pow((inferior.poses[i].pose.position.y-superior.poses[j].pose.position.y),2));
            
            if(euclideanDistance<0.5){
                ROS_INFO("Collision detected.");
                ROS_INFO("Collsion at Pose %i on Path1.", j);
                ROS_INFO("Collsion at Pose %i on Path2.", i);
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
 
    int timeBtwPoses = inferior.poses[collisionPose].header.stamp.sec - superior.poses[collisionPose].header.stamp.sec; 
    
    if(timeBtwPoses > 1){
        ROS_INFO("Duration btw Poses is enough.Go!");
        return true;
    } else {
        ROS_INFO("Can't go there. We need a new plan.");
        ROS_INFO("Collsion at Time %i on Path1.", superior.poses[collisionPose].header.stamp.sec);
        ROS_INFO("Collsion at Time %i on Path2.", inferior.poses[collisionPose].header.stamp.sec);
        ROS_INFO("%i seconds between the Poses.", timeBtwPoses);
        return false;
    }
    
}
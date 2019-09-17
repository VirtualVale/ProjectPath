#include "ros/ros.h"
#include "chronos/collision_service.h"
#include "geometry_msgs/PoseStamped.h"

std::vector<geometry_msgs::PoseStamped> distance_check(nav_msgs::Path path_sup, nav_msgs::Path path_inf);
bool time_check(geometry_msgs::PoseStamped pose_sup, geometry_msgs::PoseStamped pose_inf);
float euclideanDistance(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b);

bool check4Collisions(chronos::collision_service::Request &req, chronos::collision_service::Response &res){
    std::vector<geometry_msgs::PoseStamped> collisions;
    collisions = distance_check(req.superior, req.inferior);
    if(collisions.empty())
    {
        res.collision = false;
        return true;
    }
    res.collision = true;
    res.firstCollision = collisions.front();
    res.lastCollision = collisions.back();
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("collision_service", check4Collisions);
    ROS_INFO("Ready to check for collisions.");

    ros::spin();
    return 0;
}

std::vector<geometry_msgs::PoseStamped> distance_check(nav_msgs::Path path_sup, nav_msgs::Path path_inf){

    std::vector<geometry_msgs::PoseStamped> collisionArray;
    for(int i=0; i<path_inf.poses.size();i++){  
        for(int j=0; j<path_sup.poses.size();j++){
            double distance = euclideanDistance(path_inf.poses[i], path_sup.poses[j]);
            if(distance < 0.096){
                if(time_check(path_sup.poses[j],path_inf.poses[i])){
                    collisionArray.push_back(path_inf.poses[i]);
                }
            }
        }
    }
    return collisionArray;
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
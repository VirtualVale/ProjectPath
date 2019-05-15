#include "nav_msgs/Path.h"
#include "ros/ros.h"

nav_msgs::Path path;

//Callback for Path
void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  path.header = msg->header;
  path.poses = msg->poses;
}

int main(int argc, char **argv)
{
    //ROS Initialization
    ros::init(argc, argv, "time_simulation");
    ros::NodeHandle n;

    ros::Subscriber path_sub = n.subscribe("/pathsim/my_nafvn_planner/plan", 1000, pathCallback);

    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("pathWithTime", 1000);
    while(ros::ok() && path.poses.size() == 0){}

    for(int i = 0; i < path.poses.size(); i++){

        double euclideanDistance = sqrt(pow((path.poses[i].pose.position.x-path.poses[i+1].pose.position.x),2)+pow((path.poses[i].pose.position.y-path.poses[i+1].pose.position.y),2));
        path.poses[i+1].header.stamp = path.poses[i].header.stamp + ros::Duration(4);
        ROS_INFO("time: [%i]", path.poses[i+1].header.stamp);
    }
    
    while(ros::ok()){
        ros::spinOnce();
        path_pub.publish(path);
    }
}

#include "ros/ros.h"
#include "time_experiments/pathsim.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_test");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<time_experiments::pathsim>("pathsim");
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path_test", 1000);
    time_experiments::pathsim psrv;

    
}
#include "ros/ros.h"
#include "time_experiments/collisionsim.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_test");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<time_experiments::collisionsim>("collisionsim");
    time_experiments::collisionsim srv;

    ros::Publisher collision_pub = n.advertise<bool>("collision_pub", 1000);
    //TODO has to be based on the path_server
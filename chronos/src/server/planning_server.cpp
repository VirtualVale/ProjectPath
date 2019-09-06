#include "ros/ros.h"
#include "chronos/createPath_planning_service.h"
#include "nav_msgs/Path.h"

class path_database
{
public:
    path_database();
    virtual ~path_database();



private:
    std::vector<nav_msgs::Path>  plan[99];
};


bool createPath(chronos::createPath_planning_service::Request &req, chronos::createPath_planning_service::Response &res)
{

    return true;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("createPath_planning_service", createPath);
    ROS_INFO("Ready to add a path to the plan.");

    ros::spin();

    return 0;
}
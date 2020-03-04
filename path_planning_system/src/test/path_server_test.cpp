#include "ros/ros.h"
#include "path_planning_system/path_service.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_test");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<path_planning_system::path_service>("path_service");
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path_test", 1000);
    path_planning_system::path_service psrv;

    //input check
    if(argc != 5 || isdigit(atoi(argv[1])) || isdigit(atoi(argv[2])) || isdigit(atoi(argv[3])) || isdigit(atoi(argv[4]))){
        ROS_INFO("Input Parameters are wrong!\nstart position:\nx\ny\ngoal position:\nx\ny");
        return 1;
    }

    //server call
    psrv.request.start.header.frame_id = "map";
    psrv.request.start.pose.position.x = atoi(argv[1]);
    psrv.request.start.pose.position.y = atoi(argv[2]);
    psrv.request.start.pose.orientation.w = 1.0;

    psrv.request.goal.header.frame_id = "map";
    psrv.request.goal.pose.position.x = atoi(argv[3]);
    psrv.request.goal.pose.position.y = atoi(argv[4]);
    psrv.request.goal.pose.orientation.w = 1.0;

    if(client.call(psrv))
    {
        ROS_INFO("Path size: %lu\nPath published on the '/path_test' topic.\n", psrv.response.path.poses.size());

    } else {
        ROS_INFO("Failed");
        return 1;
    }

    while(ros::ok()){
        path_pub.publish(psrv.response.path);
    }
    return 0;    

    
}
using namespace std;

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
geometry_msgs::PoseStamped start;

//Callback for Start (pose)
void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  start.pose.position.x = msg->pose.pose.position.x;
  start.pose.position.y = msg->pose.pose.position.y;
  start.pose.orientation.x = msg->pose.pose.orientation.x;
  start.pose.orientation.y = msg->pose.pose.orientation.y;
  start.pose.orientation.w = msg->pose.pose.orientation.w;
  ROS_INFO("start x: [%d]", (int) msg->pose.pose.position.x);
  ROS_INFO("start msg->x: [%d]", (int) start.pose.position.x);
}

int main(int argc,char** argv){

    ros::init(argc, argv, "pathsim");
    ros::NodeHandle n;


    tf::TransformListener tf(ros::Duration(10));
    costmap_2d::Costmap2DROS costmap ("costmap", tf);

    costmap.start();    

    navfn::NavfnROS navfn;
    navfn.initialize("my_nafvn_planner", &costmap);

    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("Pathsim", 1000);
    nav_msgs::Path path;
    std::vector<geometry_msgs::PoseStamped> plan;

    ros::Subscriber start_sub = n.subscribe("/amcl_pose", 1000, startCallback);

    double goalX = 6.0;
    double goalY = -5.0;

        start.header.frame_id = "map";
        start.header.stamp = ros::Time::now();
        start.pose.position.x = -3.0;
        start.pose.position.y = 1.0;
        start.pose.position.z = 0.0;
        start.pose.orientation.x = 0.0;
        start.pose.orientation.y = 0.0;
        start.pose.orientation.z = 0.0;
        start.pose.orientation.w = 1;
    
    geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = goalX;
        goal.pose.position.y = goalY;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 0.0;
        goal.pose.orientation.w = 1;

    navfn.makePlan(start, goal, plan);
    ROS_INFO("Size of path = %d",(int) plan.size());
    
    path.poses = plan;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";

    while(ros::ok()){
        navfn.makePlan(start, goal, plan);
        path.poses = plan;
        path_pub.publish(path);
        ros::spinOnce();
    }
    return 0;
}
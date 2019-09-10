#include "ros/ros.h"
#include "chronos/plan.h"
#include "nav_msgs/Path.h"
#include "ros/time.h"
#include <std_msgs/Time.h>

geometry_msgs::PoseStamped poseAtTime(ros::Time currentTime, nav_msgs::Path currentPath);
nav_msgs::Path pathAtTime(ros::Time currentTime, std::vector<nav_msgs::Path> plan);
nav_msgs::Path pathSlicing(ros::Time currentTime, nav_msgs::Path currentPath, bool forward);
void visualizePlan();


std::vector<nav_msgs::Path> plan[3];
ros::Time snapshot;

void planCallback(const chronos::plan::ConstPtr& msg)
{
    ROS_INFO("planCallback");
    plan[0] = msg -> plan_1;
    plan[1] = msg -> plan_2;
    plan[2] = msg -> plan_3;
    //plan[(*msg).resource_number] = (*msg).resource_plan;
}

void timeCallback(const std_msgs::Time::ConstPtr& msg)
{
    ROS_INFO("timeCallback");
    snapshot = ros::Time((*msg).data.toSec());
    ROS_INFO("new time toSec %lf", snapshot.toSec());
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "visualization");
    ros::NodeHandle n;
    ros::Subscriber plan_sub = n.subscribe("plan", 1000, planCallback);
    ros::Subscriber time_sub = n.subscribe("timer", 1000, timeCallback);
    
    //REGISTER PUBLISHER
    //Publisher for the first robot
    ros::Publisher r1_pose_pub = n.advertise<geometry_msgs::PoseStamped>("r1_pose", 1000);
    ros::Publisher r1_path_f_pub = n.advertise<nav_msgs::Path>("r1_path_f", 1000);
    ros::Publisher r1_path_t_pub = n.advertise<nav_msgs::Path>("r1_path_t", 1000);

    //Publisher for the second robot
    ros::Publisher r2_pose_pub = n.advertise<geometry_msgs::PoseStamped>("r2_pose", 1000);
    ros::Publisher r2_path_f_pub = n.advertise<nav_msgs::Path>("r2_path_f", 1000);
    ros::Publisher r2_path_t_pub = n.advertise<nav_msgs::Path>("r2_path_t", 1000);

    //Publisher for the third robot
    ros::Publisher r3_pose_pub = n.advertise<geometry_msgs::PoseStamped>("r3_pose", 1000);
    ros::Publisher r3_path_f_pub = n.advertise<nav_msgs::Path>("r3_path_f", 1000);
    ros::Publisher r3_path_t_pub = n.advertise<nav_msgs::Path>("r3_path_t", 1000);    
    ROS_INFO("pub rdy");

    while (ros::ok())
    {
      if(!plan[0].empty())
      {
          r1_path_f_pub.publish(pathSlicing(snapshot, pathAtTime(snapshot, plan[0]), true));
          r1_path_t_pub.publish(pathSlicing(snapshot, pathAtTime(snapshot, plan[0]), false));
          r1_pose_pub.publish(poseAtTime(snapshot, pathAtTime(snapshot, plan[0])));
      }
      if(!plan[1].empty())
      {
          r2_path_f_pub.publish(pathSlicing(snapshot, pathAtTime(snapshot, plan[1]), true));
          r2_path_t_pub.publish(pathSlicing(snapshot, pathAtTime(snapshot, plan[1]), false));
          r2_pose_pub.publish(poseAtTime(snapshot, pathAtTime(snapshot, plan[1])));
      }
      if(!plan[2].empty())
      {
          r3_path_f_pub.publish(pathSlicing(snapshot, pathAtTime(snapshot, plan[2]), true));
          r3_path_t_pub.publish(pathSlicing(snapshot, pathAtTime(snapshot, plan[2]), false));
          r3_pose_pub.publish(poseAtTime(snapshot, pathAtTime(snapshot, plan[2])));
      }
      ros::spinOnce();
    }
    

    return 0;
}

/*
void visualizePlan()
{
    if(!plan[0].empty())
    {
        r1_path_f_pub.publish(pathSlicing(snapshot, pathAtTime(snapshot, plan[0]), true));
        r1_path_t_pub.publish(pathSlicing(snapshot, pathAtTime(snapshot, plan[0]), false));
        r1_pose_pub.publish(poseAtTime(snapshot, pathAtTime(snapshot, plan[0]));)

    }
}
*/

geometry_msgs::PoseStamped poseAtTime(ros::Time currentTime, nav_msgs::Path currentPath)
{
  geometry_msgs::PoseStamped  searchedPose;
  double diff, optDiff = 100.00;
  for(int i=0; i<currentPath.poses.size(); i++){
    diff = abs(currentTime.toSec() - currentPath.poses[i].header.stamp.toSec());
    if(diff < optDiff){
      optDiff = diff;
      searchedPose = currentPath.poses[i];
    }
      //ROS_INFO("for-Loop(currentPose): i = %i, optDiff = %lf, diff = %lf, poseID = %i", i, optDiff, diff, poseID);
  }
  return searchedPose;
}

nav_msgs::Path pathAtTime(ros::Time currentTime, std::vector<nav_msgs::Path> plan)
{
   nav_msgs::Path searchedPath;
   double diff, optDiff = 100.00;
   for(int i=0; i<plan.size(); ++i){
     for(int j=0; j<plan[i].poses.size(); ++j){
       diff = abs(currentTime.toSec() - plan[i].poses[j].header.stamp.toSec());
       if(diff < optDiff){
         optDiff = diff;
         searchedPath = plan[i];
       }
     }
     //ROS_INFO("for-Loop(currentPath) i = %i, optDiff = %lf, diff = %lf, pathID = %i", i, optDiff, diff, pathID);
   }
   return searchedPath;
}

nav_msgs::Path pathSlicing(ros::Time currentTime, nav_msgs::Path currentPath, bool forward)
{
    int poseID = 0;
    double diff, optDiff = 100.00;
    for(int i=0; i<currentPath.poses.size(); i++){
      diff = abs(currentTime.toSec() - currentPath.poses[i].header.stamp.toSec());
      if(diff < optDiff){
        optDiff = diff;
        poseID = i;
      }
      //ROS_INFO("for-Loop(currentPose): i = %i, optDiff = %lf, diff = %lf, poseID = %i", i, optDiff, diff, poseID);
    }
    nav_msgs::Path path_forward;
    path_forward.header.frame_id = "map";
    nav_msgs::Path path_traveled;
    path_traveled.header.frame_id = "map";

    for(int i=0; i<currentPath.poses.size(); i++)
    {
      if(i <= poseID)
      {
        path_traveled.poses.push_back(currentPath.poses[i]);
      }
      if(i >= poseID)
      {
        path_forward.poses.push_back(currentPath.poses[i]);
      }
    }
    if(forward)
    {
      return path_forward;
    }
    else
    {
      return path_traveled;
    }
}
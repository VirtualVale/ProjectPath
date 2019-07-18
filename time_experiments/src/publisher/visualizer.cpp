#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "ros/time.h"
#include "std_msgs/Time.h"

ros::Time snapshot;
std::vector<nav_msgs::Path> r1_plan;
std::vector<nav_msgs::Path> r2_plan;
std::vector<nav_msgs::Path> r3_plan;

int time_2_poseID(ros::Time snapshot, std::vector<nav_msgs::Path> vector, int* position);

void timeCallback(const std_msgs::Time::ConstPtr& msg)
{
    snapshot = ros::Time((*msg).data.toSec());
    ROS_INFO("new time toSec %lf", snapshot.toSec());
    ROS_INFO("new time sec %d", snapshot.sec);
    ROS_INFO("new time nsec %d", snapshot.nsec);   
}

void r1_planCallback(const std_msgs::Vector<nav_msgs::Path>::ConstPtr& msg)
{
    r1_plan = *msg;
    ROS_INFO("r1_plan: %i", r1_plan.size());
}
void r2_planCallback(const std_msgs::Vector<nav_msgs::Path>::ConstPtr& msg)
{
    r2_plan = *msg;
    ROS_INFO("r2_plan: %i", r2_plan.size());
}
void r3_planCallback(const std_msgs::Vector<nav_msgs::Path>::ConstPtr& msg)
{
    r3_plan = *msg;
    ROS_INFO("r3_plan: %i", r3_plan.size());
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "visualizer");
    ros::NodeHandle n;

    ros::Subscriber time_sub = n.subscribe("timer", 1000, timeCallback);
    ros::Subscriber r1_sub = n.subscribe("r1_plan", 1000, r1_planCallback);
    ros::Subscriber r2_sub = n.subscribe("r2_plan", 1000, r2_planCallback);
    ros::Subscriber r3_sub = n.subscribe("r3_plan", 1000, r3_planCallback);

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

        //debug and maybe how to get the position
    int r1_position[2], r2_position[2], r3_position[2];
    //ROS_INFO("Current pose at time %.2lf: x [%.2lf], y [%.2lf], path [%i], pose [%i]", snapshot.toSec(), plan_r1[r1_position[0]].poses[r1_position[1]].pose.position.x, plan_r1[r1_position[0]].poses[r1_position[1]].pose.position.y, r1_position[0], r1_position[1]);
    
    //go through at first all paths/poses included in the vector
    while(ros::ok()){

        //ROS_INFO("time before snapshotting %lf", snapshot.toSec());
        time_2_poseID(snapshot ,plan_r1, r1_position);
        time_2_poseID(snapshot ,plan_r2, r2_position);
        time_2_poseID(snapshot ,plan_r3, r3_position);

        r1_path_f_pub.publish(plan_r1[r1_position[0]]);
        r1_pose_pub.publish(plan_r1[r1_position[0]].poses[r1_position[1]]); 
        
        r2_path_f_pub.publish(plan_r2[r2_position[0]]);
        r2_pose_pub.publish(plan_r2[r2_position[0]].poses[r2_position[1]]);

        r3_path_f_pub.publish(plan_r3[r3_position[0]]);
        r3_pose_pub.publish(plan_r3[r3_position[0]].poses[r3_position[1]]);

        ros::spinOnce();

    }

}

//search for the position of a pose element at a certain time
int time_2_poseID(ros::Time snapshot, std::vector<nav_msgs::Path> vector, int* position){

    for(int i = 0; i<vector.size(); i++){
        for(int j = 0; j<vector[i].poses.size(); j++){
            if(vector[i].poses[j].header.stamp > snapshot){

                //ROS_INFO("currentPosition %.2lf %.2lf at time %.2lf", vector[i].poses[j-1].pose.position.x, vector[i].poses[j-1].pose.position.y, vector[i].poses[j-1].header.stamp);
                position[0] = i;
                position[1] = j;
                return 1;

            }
        }
    }
    return 0;


}
#include "ros/ros.h"
#include "time_experiments/collisionsim.h"
#include "time_experiments/timesim.h"
#include "time_experiments/pathsim.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_test");
    ros::NodeHandle n;

    //client for collision detection: csrv
    ros::ServiceClient cclient = n.serviceClient<time_experiments::collisionsim>("collisionsim");
    time_experiments::collisionsim csrv;

    //client for timestamping: tsrv
    ros::ServiceClient tclient = n.serviceClient<time_experiments::timesim>("timesim");
    time_experiments::timesim tsrv;

    //client for pathplanning
    ros::ServiceClient pclient = n.serviceClient<time_experiments::pathsim>("pathsim");
    time_experiments::pathsim psrv;

    //Creating the first path
    nav_msgs::Path path1;

    psrv.request.start.header.frame_id = "map";
    psrv.request.start.header.stamp = ros::Time::now();
    psrv.request.start.pose.position.x = atoi(argv[1]);
    psrv.request.start.pose.position.y = atoi(argv[2]);
    psrv.request.start.pose.orientation.w = 1.0;

    psrv.request.goal.header.frame_id = "map";
    psrv.request.goal.header.stamp = ros::Time::now();
    psrv.request.goal.pose.position.x = atoi(argv[3]);
    psrv.request.goal.pose.position.y = atoi(argv[4]);
    psrv.request.goal.pose.orientation.w = 1.0;

    if(pclient.call(psrv)){
        path1 = psrv.response.path;
        ROS_INFO("Path (1) size: %lu", psrv.response.path.poses.size());
    } else {
        ROS_INFO("Pathplanning 1 failed!");
    }

    //Creating a second path
    nav_msgs::Path path2;


    psrv.request.start.pose.position.x = atoi(argv[5]);
    psrv.request.start.pose.position.y = atoi(argv[6]);
    psrv.request.start.pose.orientation.w = 1.0;

    psrv.request.goal.pose.position.x = atoi(argv[7]);
    psrv.request.goal.pose.position.y = atoi(argv[8]);
    psrv.request.goal.pose.orientation.w = 1.0;

    if(pclient.call(psrv)){
        path2 = psrv.response.path;
        ROS_INFO("Path (2) size: %lu", psrv.response.path.poses.size());
    } else {
        ROS_INFO("Pathplanning 2 failed!");
    }

    ros::Time startTime1 (100, 0);
    path1.poses[0].header.stamp = startTime1;

    tsrv.request.original_path = path1;
    tsrv.request.average_velocity = atoi(argv[9]) + (0.1*atoi(argv[10]));

    
    if(tclient.call(tsrv)){
        path1 = tsrv.response.timesim_path;
        
        ROS_INFO("Path (1) last time: %i", tsrv.response.timesim_path.poses[tsrv.response.timesim_path.poses.size()-1].header.stamp.sec);
    } else {
      ROS_INFO("Timestamping 1 failed!");
      return 1;
    }


    path2.poses[0].header.stamp = startTime1;

    tsrv.request.original_path = path2;
    tsrv.request.average_velocity = atoi(argv[11]) + (0.1*atoi(argv[12]));

    if(tclient.call(tsrv)){
        path2 = tsrv.response.timesim_path;
        ROS_INFO("Path (2) last time: %i", tsrv.response.timesim_path.poses[tsrv.response.timesim_path.poses.size()-1].header.stamp.sec);
    } else {
      ROS_INFO("Timestamping 2 failed!");
      return 1;
    }

    csrv.request.superior = path1;
    csrv.request.inferior = path2;

    if(cclient.call(csrv)){
        if(csrv.response.collision.header.seq == -1 ){
            ROS_INFO("Seq.nr. : %i", csrv.response.collision.header.seq);
            ROS_INFO("No Collision");
        } else {
            ROS_INFO("Seq.nr. : %i", csrv.response.collision.header.seq);
            ROS_INFO("Collision at Time: %i", csrv.response.collision.header.stamp.sec);
            ROS_INFO("Collision at Coordinates: %f, %f", csrv.response.collision.pose.position.x, csrv.response.collision.pose.position.y);
        }

    } else {
        ROS_INFO("Collisioncheck failed!");
        //return 1;
    }
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path", 1000);
    ros::Publisher path2_pub = n.advertise<nav_msgs::Path>("path2", 1000);
    ros::Publisher collision_pub = n.advertise<geometry_msgs::PoseStamped>("collision", 1000);
    
    while(ros::ok()){
        path_pub.publish(path1);
        path2_pub.publish(path2);
        csrv.response.collision.header.frame_id = "map"; //TODO remove
        if(csrv.response.collision.header.seq != -1 )collision_pub.publish(csrv.response.collision);
    }
    return 0;
}
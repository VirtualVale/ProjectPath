#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "chronos/planningAction.h"
#include "geometry_msgs/PoseStamped.h"

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    actionlib::SimpleActionClient<chronos::planningAction> ac("planning", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    chronos::planningGoal goal;
    goal.resource_number = (*msg).header.seq;
    ROS_INFO("resource_number -> seq %i", (*msg).header.seq);
    goal.goal.pose.position.x = (*msg).pose.position.x;
    goal.goal.pose.position.y = (*msg).pose.position.y;
    goal.start_time.data = (*msg).header.stamp;

    ROS_INFO("start time: %lf , %lf", goal.start_time.data.toSec(), (*msg).header.stamp.toSec());
    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");
    
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "planning_test");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("planning_test", 1000, goalCallback);
    actionlib::SimpleActionClient<chronos::planningAction> ac("planning_actionserver", true);
    
    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    chronos::planningGoal goal;
    goal.task = 0;
    goal.resource_number = 1;
    goal.goal.pose.position.x = 0;
    goal.goal.pose.position.y = 5.5;
    goal.start_time.data = ros::Time(100);

    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    ros::spin();
    return 0;
}
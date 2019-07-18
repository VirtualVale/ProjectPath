#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <time_experiments/PTSAction.h>
#include "nav_msgs/Path.h"
#include "ros/time.h"
#include "time_experiments/pathsim.h"
#include "time_experiments/timesim.h"

std::vector<nav_msgs::Path> path_2_vector(std::vector<nav_msgs::Path> vector, float x1, float y1, float x2, float y2, ros::Time time, ros::ServiceClient path_client, ros::ServiceClient time_client);
int time_2_poseID(ros::Time snapshot, std::vector<nav_msgs::Path> vector, int* position);

class PTSAction
{
protected:

  ros::NodeHandle n;
  actionlib::SimpleActionServer<time_experiments::PTSAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  time_experiments::PTSFeedback feedback_;
  time_experiments::PTSResult result_;

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

  //data generation
  ros::Time snapshot = ros::Time(115.00);

  //vector(path) for each robot
  std::vector<nav_msgs::Path> r1_plan;
  std::vector<nav_msgs::Path> r2_plan;
  std::vector<nav_msgs::Path> r3_plan;

  //clients for the pathplanning and timestamping
  ros::ServiceClient path_client = n.serviceClient<time_experiments::pathsim>("pathsim");
  ros::ServiceClient time_client = n.serviceClient<time_experiments::timesim>("timesim");


public:

  PTSAction(std::string name) :
    as_(n, name, boost::bind(&PTSAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~PTSAction(void)
  {
  }

  void executeCB(const time_experiments::PTSGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    switch (goal->resource_number)
    {
      case 1:
        r1_plan = path_2_vector(r1_plan, 0, 0, goal->goal.pose.position.x, goal->goal.pose.position.y, goal->goal.header.stamp, path_client, time_client);
        break;
      case 2:
        r2_plan = path_2_vector(r2_plan, 0, 1, goal->goal.pose.position.x, goal->goal.pose.position.y, goal->goal.header.stamp, path_client, time_client);
        break;
      case 3:
        r3_plan = path_2_vector(r3_plan, 0, 2, goal->goal.pose.position.x, goal->goal.pose.position.y, goal->goal.header.stamp, path_client, time_client);
        break;
      default:
        success = false;
    }
    
    ROS_INFO("r1_plan: %i, r2_plan: %i, r3_plan: %i", r1_plan.size(), r2_plan.size(), r3_plan.size());

    /*
    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
    */
  }



  void testFunction(){

    //plan for r1
    r1_plan = path_2_vector(r1_plan, 0, 0, 0, 5.5, ros::Time(100.00), path_client, time_client);
    r1_plan = path_2_vector(r1_plan, 0, 5.5, 6, 5.5, ros::Time(110.00), path_client, time_client);
    r1_plan = path_2_vector(r1_plan, 6, 5.5, 0, 0, ros::Time(120.00), path_client, time_client);
    ROS_INFO("r1_plan: %lu", r1_plan.size()); //for debugging

    //plan for r2
    r2_plan = path_2_vector(r2_plan, 0, 1, 3.5, 3.5, ros::Time(110.00), path_client, time_client);
    r2_plan = path_2_vector(r2_plan, 3.5, 3.5, 5.5, 3.5, ros::Time(120.00), path_client, time_client);
    r2_plan = path_2_vector(r2_plan, 5.5, 3.5, 0, 1, ros::Time(130.00), path_client, time_client);

    //plan for r3
    r3_plan = path_2_vector(r3_plan, 0, 2, 5.5, 3.5, ros::Time(120.00), path_client, time_client);
    r3_plan = path_2_vector(r3_plan, 5.5, 3.5, 0, 2, ros::Time(130.00), path_client, time_client);

      int r1_position[2], r2_position[2], r3_position[2];

      //ROS_INFO("time before snapshotting %lf", snapshot.toSec());
      time_2_poseID(snapshot ,r1_plan, r1_position);
      time_2_poseID(snapshot ,r2_plan, r2_position);
      time_2_poseID(snapshot ,r3_plan, r3_position);

      r1_path_f_pub.publish(r1_plan[r1_position[0]]);
      r1_pose_pub.publish(r1_plan[r1_position[0]].poses[r1_position[1]]); 
        
      r2_path_f_pub.publish(r2_plan[r2_position[0]]);
      r2_pose_pub.publish(r2_plan[r2_position[0]].poses[r2_position[1]]);

      r3_path_f_pub.publish(r3_plan[r3_position[0]]);
      r3_pose_pub.publish(r3_plan[r3_position[0]].poses[r3_position[1]]);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "PTS");

  PTSAction PTS("PTS");
  while(ros::ok()){
    //PTS.testFunction();
    ros::spinOnce();
  }


  return 0;
}

  //function for pathplanning and timestamping service call, gives back the updated vector
  std::vector<nav_msgs::Path> path_2_vector(std::vector<nav_msgs::Path> vector, float x1, float y1, float x2, float y2, ros::Time time, ros::ServiceClient path_client, ros::ServiceClient time_client){

      //pathplanning call with parameters
      time_experiments::pathsim psrv;
      psrv.request.start.header.frame_id = "map";
      psrv.request.start.header.stamp = time;
      psrv.request.start.pose.position.x = x1;
      psrv.request.start.pose.position.y = y1;
      psrv.request.start.pose.orientation.w = 1.0;
      psrv.request.goal.header.frame_id = "map";
      //psrv.request.goal.header.stamp = ros::Time::now();
      psrv.request.goal.pose.position.x = x2;
      psrv.request.goal.pose.position.y = y2;
      psrv.request.goal.pose.orientation.w = 1.0;
      if(path_client.call(psrv))
      {
          ROS_INFO("Path size: %lu", psrv.response.path.poses.size());
      } else {
          ROS_INFO("Failed");
      }

      //timestamping
      time_experiments::timesim tsrv;
      tsrv.request.original_path = psrv.response.path;
      tsrv.request.original_path.poses.front().header.stamp = time;
      tsrv.request.average_velocity = 0.5;
      if(time_client.call(tsrv)){
        ROS_INFO("start time: %.2lf, end time: %.2lf", tsrv.response.timesim_path.poses.front().header.stamp.toSec(), tsrv.response.timesim_path.poses.back().header.stamp.toSec());
      } else {
        ROS_INFO("Failed");
      }

      vector.push_back(tsrv.response.timesim_path);
      ROS_INFO("vector size: %lu", vector.size());
      return vector;
  }

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
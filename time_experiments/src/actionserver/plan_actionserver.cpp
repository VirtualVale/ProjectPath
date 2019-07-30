#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <time_experiments/PTSAction.h>
#include "nav_msgs/Path.h"
#include "ros/time.h"
#include "time_experiments/pathsim.h"
#include "time_experiments/timesim.h"
#include "time_experiments/collisionsim.h"

ros::Time snapshot;

std::vector<nav_msgs::Path> path_2_vector(std::vector<nav_msgs::Path> vector, float x1, float y1, float x2, float y2, ros::Time time, ros::ServiceClient path_client, ros::ServiceClient time_client);
int time_2_poseID(ros::Time snapshot, std::vector<nav_msgs::Path> vector, int* position);
int currentPose(ros::Time currentTime, nav_msgs::Path currentPath);
int currentPath(ros::Time currentTime, std::vector<nav_msgs::Path> plan);
int currentPosition(ros::Time snapshot, std::vector<nav_msgs::Path> vector, int* position);

void timeCallback(const std_msgs::Time::ConstPtr& msg)
{
    snapshot = ros::Time((*msg).data.toSec());
    ROS_INFO("new time toSec %lf", snapshot.toSec());
    ROS_INFO("new time sec %d", snapshot.sec);
    ROS_INFO("new time nsec %d", snapshot.nsec);
}

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

  //vector(path) for each robot
  std::vector<nav_msgs::Path> r1_plan;
  std::vector<nav_msgs::Path> r2_plan;
  std::vector<nav_msgs::Path> r3_plan;

  //overall plan
  std::vector<std::vector<nav_msgs::Path> > full_plan;
  
  //clients for the pathplanning and timestamping
  ros::ServiceClient path_client = n.serviceClient<time_experiments::pathsim>("pathsim");
  ros::ServiceClient time_client = n.serviceClient<time_experiments::timesim>("timesim");
  ros::ServiceClient collision_client = n.serviceClient<time_experiments::collisionsim>("collisionsim");

public:

  PTSAction(std::string name) :
    as_(n, name, boost::bind(&PTSAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    full_plan.push_back(r1_plan);
    full_plan.push_back(r2_plan);
    full_plan.push_back(r3_plan);
  }

  ~PTSAction(void)
  {
  }

  void executeCB(const time_experiments::PTSGoalConstPtr &goal)
  {
    // helper variables
    bool success = true;

    //RESOURCE
    //Resources 1,2,3 are mapped to 0,1,2
    int resource = (goal-> resource_number) - 1;

    //START TIME
    ros::Time startTime = goal->start_time.data;

    //START POSITION
    double startX;
    double startY; 
    //no paths registered
    if(full_plan[resource].size() == 0){
      ROS_INFO("START POSITION: first path is initialized");
      switch (resource)
      {
        case 0:
          startX = 0;
          startY = 0;
          break;
        case 1:
          startX = 0;
          startY = 1;
          break;
        case 2:
          startX = 0;
          startY = 2;
          break;
        default:
          ROS_INFO("Error: Invalid resource number");
      }
    } else {
      int pathId = currentPath(startTime, full_plan[resource]);
      //time is between zero time and last path
      startX = full_plan[resource][pathId].poses.back().pose.position.x;
      startY = full_plan[resource][pathId].poses.back().pose.position.y;
    }

    //GOAL POSITION
    double goalX = goal->goal.pose.position.x;
    double goalY = goal->goal.pose.position.y;

    //PATHCREATION AND TIMESTAMPING + ADDING TO THE PLAN
    ROS_INFO("planning resources: resource_nr %i, start %.2lf:%.2lf, goal %.2lf:%.2lf, time: %.2lf", resource,startX, startY, goalX, goalY, startTime);
    full_plan[resource] = path_2_vector(full_plan[resource],startX, startY, goalX, goalY, startTime, path_client, time_client);
    
    //COLLISIONCHECKING
    time_experiments::collisionsim csrv;
    csrv.request.superior = full_plan[resource].back();

    for(int i = resource+1; i<full_plan.size(); i++){
      for(int j = 0; j<full_plan[i].size(); j++){
        
        csrv.request.inferior = full_plan[i][j];
        ROS_INFO("COLLISIONCHECKING: resource: %i, Pathnr: %i", i, j);
        if(collision_client.call(csrv)){
            ROS_INFO("Collision at Time: %i", csrv.response.collision.header.stamp.sec);
            ROS_INFO("Collision at Coordinates: %f, %f", csrv.response.collision.pose.position.x, csrv.response.collision.pose.position.y);
        } else {
            ROS_INFO("Paths do not collide!");
            
        }
      }
    }
    for(int i = resource-1; i>=0; i--){
      for(int j = 0; j<full_plan[i].size(); j++){
        
        csrv.request.inferior = full_plan[i][j];
        ROS_INFO("COLLISIONCHECKING: resource: %i, Pathnr: %i", i, j);
        if(collision_client.call(csrv)){
            ROS_INFO("Collision at Time: %i", csrv.response.collision.header.stamp.sec);
            ROS_INFO("Collision at Coordinates: %f, %f", csrv.response.collision.pose.position.x, csrv.response.collision.pose.position.y);
        } else {
            ROS_INFO("Paths do not collide!");
            
        }
      }
    }
    
    as_.setSucceeded(result_);
    
  }

  void publishPlan(){
         
        nav_msgs::Path r1_currentPath, r2_currentPath, r3_currentPath;

        if(!full_plan[0].empty()){
          r1_currentPath = full_plan[0][currentPath(snapshot, full_plan[0])];
          r1_path_f_pub.publish(r1_currentPath);
          r1_pose_pub.publish(r1_currentPath.poses[currentPose(snapshot, r1_currentPath)]);
        }

        if(!full_plan[1].empty()){
          r2_currentPath = full_plan[1][currentPath(snapshot, full_plan[1])];
          r2_path_f_pub.publish(r2_currentPath);
          r2_pose_pub.publish(r2_currentPath.poses[currentPose(snapshot, r2_currentPath)]);
        }

        if(!full_plan[2].empty()){
          r3_currentPath = full_plan[2][currentPath(snapshot, full_plan[2])];
          r3_path_f_pub.publish(r3_currentPath);
          r3_pose_pub.publish(r3_currentPath.poses[currentPose(snapshot, full_plan[2][currentPath(snapshot, full_plan[2])])]);
        }
        
        
        
  }

  //for first test
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
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("timer", 1000, timeCallback);
  snapshot = ros::Time(100);
  while(ros::ok()){
    PTS.publishPlan();
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

  //searches the current travelled path in the parameter plan
  int currentPath(ros::Time currentTime, std::vector<nav_msgs::Path> plan){

    int pathID = 0;
    double diff, optDiff = 100.00;
    for(int i=0; i<plan.size(); ++i){
      for(int j=0; j<plan[i].poses.size(); ++j){
        diff = abs(currentTime.toSec() - plan[i].poses[j].header.stamp.toSec());
        if(diff < optDiff){
          optDiff = diff;
          pathID = i;
        }
      }
      //ROS_INFO("for-Loop(currentPath) i = %i, optDiff = %lf, diff = %lf, pathID = %i", i, optDiff, diff, pathID);
    }
    return pathID;
  }

  //searches the current pose in the transferred path TODO minimize calculation time with logarithmic improvements
  int currentPose(ros::Time currentTime, nav_msgs::Path currentPath){

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
    return poseID;
  }

/*
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
  */
    //ROS_INFO("r1_plan: %lu, r2_plan: %lu, r3_plan: %lu", r1_plan.size(), r2_plan.size(), r3_plan.size());
    //ROS_INFO("TEST: r1_currentPath %lu, r1_currentPose: %lu", currentPath(snapshot, full_plan[resource]), currentPose(snapshot, full_plan[resource][currentPath(snapshot, full_plan[resource])]));
    //ROS_INFO("38 time: %lf", full_plan[resource][0].poses[38].header.stamp.toSec());

    /*while(ros::ok()){
      r1_path_f_pub.publish(r1_plan[0]);
    }*/
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
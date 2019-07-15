#include "ros/ros.h"
#include "std_msgs/Time.h"

#include <sstream>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher timer_pub = n.advertise<std_msgs::Time>("timer", 1000);
  ros::Rate loop_rate(60);
  std_msgs::Time time_count;

  while (ros::ok())
  {
    
    for(int i=0; i<3000; i++){
            
            time_count.data = ros::Time(100 + i*0.01);
            timer_pub.publish(time_count);
            ros::spinOnce();
            loop_rate.sleep();
    }

  }


  return 0;
}
#include "my_class.h" // header in local directory
#include <ros/ros.h>

using namespace N;
using namespace std;

void my_class::do_something()
{
    ROS_INFO("Doing something!");
}
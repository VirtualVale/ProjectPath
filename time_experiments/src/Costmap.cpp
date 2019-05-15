using namespace std;

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

int main(int argc,char** argv){

    ros::init(argc, argv, "pathsim");
    ros::NodeHandle n;

    tf::TransformListener tf(ros::Duration(10));
    costmap_2d::Costmap2DROS costmap ("global_costmap", tf);
    costmap.start();

    return 0;
}
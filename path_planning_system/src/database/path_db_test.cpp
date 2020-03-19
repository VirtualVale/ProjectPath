#include "path_db.h"

using namespace path_db;

int main(int argc, char **argv)
{
    ROS_INFO("Executing path_db");
    PathDB p_db;
    p_db.helloWorld();
    return 0;
}
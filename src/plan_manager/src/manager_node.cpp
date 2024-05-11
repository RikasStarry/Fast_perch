#include "/home/gnij/Fast-Perching-master/src/plan_manager/include/plan_manager.h"
#include <ros/ros.h>


int main( int argc, char * argv[] )
{ 
    ros::init(argc, argv, "manager_node");
    ros::NodeHandle nh("~");

    PlanManager plan_manager;
    plan_manager.init(nh);

    ros::spin();

    return 0;
}
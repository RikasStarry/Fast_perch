#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include "sub.h"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "sub_topic_node");
    ros::NodeHandle nh("~");
    
    SUB::subt sub_topic_temp(nh);
    ros::spin();
    return 0;
}
   
    
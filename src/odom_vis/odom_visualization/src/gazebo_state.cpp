#include "gazebo_msgs/ModelStates.h"
#include "nav_msgs/Path.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <iterator>
#include <string>
#include <unistd.h>
#include "geometry_msgs/PoseStamped.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include "nav_msgs/Odometry.h"
using namespace std;

ros::Subscriber sub_gazebo_state;
ros::Publisher pub_gazebo_path;
ros::Publisher pub_gazebo_odom;
nav_msgs::Odometry odom;
nav_msgs::Path path;
geometry_msgs::PoseStamped pose;
string model_name, model_frame;
int model_index;
string target_frame;
int counts;

void gazebo_state_cb(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    //1s检查一次model_name
    if (counts % 1000 == 0)
    {
        for (int i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == model_name)
            {
                model_index = i;
                break;
            }
            model_index = -1;
        }
        counts = 1;
    }
    else
    {
        ++counts;
    }

    if (model_index != -1)
    {
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = target_frame;
        odom.pose.pose = msg->pose[model_index];
        odom.twist.twist = msg->twist[model_index];

        pose.header.frame_id = target_frame;
        path.header.frame_id = target_frame;
        pose.header.stamp = path.header.stamp = ros::Time::now();
        pose.pose = msg->pose[model_index];
        path.poses.push_back(pose);
    }
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gazebo_state");
    ros::NodeHandle nh("~");
    //1000hz
    sub_gazebo_state = 
        nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, gazebo_state_cb);
    pub_gazebo_path = nh.advertise<nav_msgs::Path>("/gazebo_path", 10);
    pub_gazebo_odom = nh.advertise<nav_msgs::Odometry>("/gazebo_odom", 200);

    // 默认参数为地面，gazebo仿真肯定会有地面模型，避免报错
    nh.param<string>("model_name", model_name, "ground_plane");
    nh.param<string>("model_frame", model_frame, "ground_plane");
    nh.param<string>("target_frame", target_frame, "map");
    ROS_INFO_STREAM(" model_name is " << model_name);

    counts = 0;
    model_index = 0;

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        pub_gazebo_path.publish(path);
        pub_gazebo_odom.publish(odom);

        rate.sleep();
    }
    return 0;
}
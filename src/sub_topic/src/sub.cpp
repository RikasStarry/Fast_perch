#include "sub.h"

namespace SUB{
    subt::subt(ros::NodeHandle& nh):nh_(nh)
    {
        odom1 = nh_.subscribe<nav_msgs::Odometry>("/gazebo_odom", 1000, &subt::rcvOdomCallBack, this);
        imu1 = nh_.subscribe<sensor_msgs::Imu>("/iris/imu", 1000, &subt::imu1Callback, this);
        imu2 = nh_.subscribe<sensor_msgs::Imu>("/iris/ground_truth/imu", 1000, &subt::imu2Callback, this);
        real_traj = nh_.subscribe<nav_msgs::Path>("/gazebo_path", 1, &subt::traj1Callback,this);
        ideal_traj = nh_.subscribe<nav_msgs::Path>("/manager_node/traj", 1, &subt::traj2Callback,this);
        char buffer[256];
        char *val = getcwd(buffer, sizeof(buffer));
        if (val) {
            std::cout << buffer << std::endl;
        }
    }

    void subt::rcvOdomCallBack(const nav_msgs::Odometry::ConstPtr& msg) {//全部是惯性坐标系
        std::ofstream odom_file;
        odom_file.open("/home/gnij/Fast-Perching-master/src/sub_topic/txt/union/odom.txt", std::ios_base::app);
        if(odom_file.is_open())
        {   
            auto pos = msg->pose.pose.position;
            auto ori = msg->pose.pose.orientation;
            auto vel = msg->twist.twist.linear;
            auto omega = msg->twist.twist.angular;
            Eigen::Quaterniond q(ori.w,ori.x,ori.y,ori.z);
            Eigen::Vector3d eulerAngle = q.toRotationMatrix().eulerAngles(2, 1, 0);
            if(!odom1_init_valid)
            {
                odom1_init = msg->header.stamp;
                odom_file <<"0.0 ";
                odom1_init_valid = true;  
            }
            else{
                auto t = (msg->header.stamp-odom1_init).toSec();
                odom_file <<t<<" ";
            }
            
            odom_file << pos.x<<" "<< pos.y<<" "<<pos.z<<" "<<eulerAngle[0]<<" "<<eulerAngle[1]<<" "<<eulerAngle[2]<<" ";
            odom_file << vel.x<<" "<< vel.y<<" "<< vel.z<<" "<< omega.x<<" "<< omega.y<<" "<< omega.z<<"\n";
            odom_file.close();
        }
        else
        {
            ROS_ERROR("Failed to open file for writing.");
        }
    }


    void subt::imu1Callback(const sensor_msgs::ImuConstPtr& msg)//全部是机体坐标系
    {
        std::ofstream imu1_file;
        imu1_file.open("/home/gnij/Fast-Perching-master/src/sub_topic/txt/union/imu1.txt", std::ios_base::app);
        if(imu1_file.is_open())
        {   
            if(!imu1_init_valid)
            {
                imu1_init = msg->header.stamp;
                imu1_file <<"0.0 ";
                imu1_init_valid = true;  
            }
            else{
                auto t = (msg->header.stamp-imu1_init).toSec();
                imu1_file <<t<<" ";
            }
            auto acc = msg->linear_acceleration;
            auto omega = msg->angular_velocity;
            imu1_file << omega.x<<" "<< omega.y<<" "<<omega.z<<" ";
            imu1_file << acc.x<<" "<< acc.y<<" "<< acc.z<<"\n";
            imu1_file.close();
        }
        else
        {
            ROS_ERROR("Failed to open file for writing.");
        }
    }

    void subt::imu2Callback(const sensor_msgs::ImuConstPtr& msg)
    {
        std::ofstream imu2_file;
        imu2_file.open("/home/gnij/Fast-Perching-master/src/sub_topic/txt/union/imu2.txt", std::ios_base::app);
        if(imu2_file.is_open())
        {   
            if(!imu2_init_valid)
            {
                imu2_init = msg->header.stamp;
                imu2_file <<"0.0 ";
                imu2_init_valid = true;  
            }
            else{
                auto t = (msg->header.stamp-imu2_init).toSec();
                imu2_file <<t<<" ";
            }
            auto acc = msg->linear_acceleration;
            auto omega = msg->angular_velocity;
            imu2_file << omega.x<<" "<< omega.y<<" "<<omega.z<<" ";
            imu2_file << acc.x<<" "<< acc.y<<" "<< acc.z<<"\n";
            imu2_file.close();
        }
        else
        {
            ROS_ERROR("Failed to open file for writing.");
        }
    }

    void subt::traj1Callback(const nav_msgs::Path::ConstPtr& msg)
    {
        if(traj1_init_valid)
            return;
        std::ofstream traj1_file;
        traj1_file.open("/home/gnij/Fast-Perching-master/src/sub_topic/txt/union/traj1.txt", std::ios_base::app);
        if(traj1_file.is_open())
        {   
            for(int i=0;i<msg->poses.size();i++)
            {
                auto pos = msg->poses[i].pose.position;
                traj1_file << pos.x<<" "<< pos.y<<" "<<pos.z<<"\n";
            }   
        }
        else
        {
            ROS_ERROR("Failed to open file for writing.");
        }
        traj1_file.close();
        traj1_init_valid = true;
    }

    void subt::traj2Callback(const nav_msgs::Path::ConstPtr& msg)
    {
        if(traj2_init_valid)
            return;
        std::ofstream traj2_file;
        traj2_file.open("/home/gnij/Fast-Perching-master/src/sub_topic/txt/union/traj2.txt", std::ios_base::app);
        if(traj2_file.is_open())
        {   
            for(int i=0;i<msg->poses.size();i++)
            {
                auto pos = msg->poses[i].pose.position;
                traj2_file << pos.x<<" "<< pos.y<<" "<<pos.z<<"\n";
            }   
        }
        else
        {
            ROS_ERROR("Failed to open file for writing.");
        }
        traj2_file.close();
        traj2_init_valid = true;
    }
}
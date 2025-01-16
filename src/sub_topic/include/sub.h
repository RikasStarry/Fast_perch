#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Eigen>

namespace SUB{
    class subt{
    private:
        ros::NodeHandle nh_;               // 
        ros::Subscriber odom1;             // /gazebo_odom
        ros::Subscriber imu1;              // /iris/imu
        ros::Subscriber imu2;              // /iris/ground_truth/imu
        ros::Subscriber real_traj;         // /gazebo_path
        ros::Subscriber ideal_traj;        // /manager_node/traj
        ros::Time odom1_init;
        ros::Time imu1_init;
        ros::Time imu2_init;
        ros::Time traj1_init;
        ros::Time traj2_init;
        bool odom1_init_valid = false;
        bool imu1_init_valid = false;
        bool imu2_init_valid = false;
        bool traj1_init_valid = false;
        bool traj2_init_valid = false;
            
    public:
        subt(ros::NodeHandle& nh);
        ~subt() {}
        void rcvOdomCallBack(const nav_msgs::Odometry::ConstPtr& msg);
        void imu1Callback(const sensor_msgs::ImuConstPtr& msg);
        void imu2Callback(const sensor_msgs::ImuConstPtr& msg);
        void traj1Callback(const nav_msgs::Path::ConstPtr& msg);
        void traj2Callback(const nav_msgs::Path::ConstPtr& msg);
    };
}
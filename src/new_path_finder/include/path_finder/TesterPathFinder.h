#include "occ_grid/occ_map.h"
#include "path_finder/rrt_sharp.h"
#include "path_finder/rrt_star.h"
#include "path_finder/rrt.h"
#include "path_finder/brrt.h"
#include "path_finder/brrt_star.h"
#include "visualization/visualization.hpp"
#include "uneven_map/uneven_map.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <string>
namespace PF
{
    class TesterPathFinder
    {
    private:
        ros::NodeHandle nh_;
        ros::Timer execution_timer_;
        //union mod
        ros::Subscriber sub_odom;
        ros::Subscriber sub_goal;

        env::OccMap::Ptr env_ptr_;
        uneven_planner::UnevenMap::Ptr uneven_map_;
        std::shared_ptr<visualization::Visualization> vis_ptr_;
        shared_ptr<path_plan::RRTSharp> rrt_sharp_ptr_;
        shared_ptr<path_plan::RRTStar> rrt_star_ptr_;
        shared_ptr<path_plan::RRT> rrt_ptr_;
        shared_ptr<path_plan::BRRT> brrt_ptr_;
        shared_ptr<path_plan::BRRTStar> brrt_star_ptr_;

        Eigen::Vector3d start_, goal_,upper_goal;
        bool has_path,has_start;

        bool run_rrt_, run_rrt_star_, run_rrt_sharp_;
        bool run_brrt_, run_brrt_star_;
        //bool GUILD,Informed;
        //union mod
        bool start_recv=false;
        bool goal_recv=false;
        
    public:    
        TesterPathFinder(const ros::NodeHandle &nh,const env::OccMap::Ptr env_ptr,const uneven_planner::UnevenMap::Ptr uneven_map);

        ~TesterPathFinder(){};

        void rcvOdomCallBack(const nav_msgs::Odometry::ConstPtr& msgPtr);

        void rcvGoalCallBack(const geometry_msgs::Point::ConstPtr& msgPtr);

        void calInitPath();
        void executionCallback(const ros::TimerEvent &event);
        void handle_finalpath(vector<Eigen::Vector3d>& v);
    };
}

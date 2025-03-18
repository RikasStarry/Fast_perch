#include "path_finder/TesterPathFinder.h"
namespace PF
{
    TesterPathFinder::TesterPathFinder(const ros::NodeHandle &nh,const env::OccMap::Ptr env_ptr,const uneven_planner::UnevenMap::Ptr uneven_map) : nh_(nh),env_ptr_(env_ptr),uneven_map_(uneven_map)
    {
        vis_ptr_ = std::make_shared<visualization::Visualization>(nh_);
        vis_ptr_->registe<visualization_msgs::Marker>("start");
        vis_ptr_->registe<visualization_msgs::Marker>("goal");

        rrt_sharp_ptr_ = std::make_shared<path_plan::RRTSharp>(nh_, env_ptr_);
        rrt_sharp_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("rrt_sharp_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("rrt_sharp_final_wpts");

        rrt_star_ptr_ = std::make_shared<path_plan::RRTStar>(nh_, env_ptr_,uneven_map_);
        rrt_star_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("rrt_star_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("rrt_star_final_wpts");
        vis_ptr_->registe<visualization_msgs::MarkerArray>("rrt_star_paths");

        rrt_ptr_ = std::make_shared<path_plan::RRT>(nh_, env_ptr_);
        rrt_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("rrt_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("rrt_final_wpts");

        brrt_ptr_ = std::make_shared<path_plan::BRRT>(nh_, env_ptr_);
        brrt_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("brrt_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("brrt_final_wpts");

        brrt_star_ptr_ = std::make_shared<path_plan::BRRTStar>(nh_, env_ptr_);
        brrt_star_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("brrt_star_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("brrt_star_final_wpts");

        nh_.param("path_finder/run_rrt", run_rrt_, false);
        nh_.param("path_finder/run_rrt_star", run_rrt_star_, true);
        nh_.param("path_finder/run_rrt_sharp", run_rrt_sharp_, false);
        nh_.param("path_finder/run_brrt", run_brrt_, false);
        nh_.param("path_finder/run_brrt_star", run_brrt_star_, false);
        // nh_.param("use_informed_sampling", Informed, true);
        // nh_.param("use_GUILD_sampling", GUILD, false);
        
        execution_timer_ = nh_.createTimer(ros::Duration(1), &TesterPathFinder::executionCallback, this);
        start_.setZero();
        sub_odom = nh_.subscribe("/gazebo_odom", 1, &TesterPathFinder::rcvOdomCallBack, this);
        sub_goal = nh_.subscribe("uneven_goal", 1, &TesterPathFinder::rcvGoalCallBack, this);
    }

    void TesterPathFinder::rcvOdomCallBack(const nav_msgs::Odometry::ConstPtr& msgPtr) {
        start_ << msgPtr->pose.pose.position.x, msgPtr->pose.pose.position.y, msgPtr->pose.pose.position.z;
        //ROS_INFO_STREAM("\n-----------------------------\nstart is at " << start_.transpose());
        start_recv = true;
    }

    void TesterPathFinder::rcvGoalCallBack(const geometry_msgs::Point::ConstPtr& msgPtr) {
        goal_<<msgPtr->x,msgPtr->y,msgPtr->z;
        upper_goal = goal_;
        while(!env_ptr_->isStateValid(upper_goal))
        {
            upper_goal[2]+=0.1;
        }
        ROS_INFO_STREAM("\n-----------------------------\ngoal is at " << upper_goal.transpose());
        rrt_star_ptr_->setBottom(goal_);
        goal_recv = true;
        calInitPath();
        goal_recv = false;
    }

    void TesterPathFinder::calInitPath()
    {
        start_<<-4.0,-4.0,2.0;
        vis_ptr_->visualize_a_ball(start_, 0.1, "start", visualization::Color::pink);
        vis_ptr_->visualize_a_ball(upper_goal, 0.1, "goal", visualization::Color::steelblue);

        // BiasSampler sampler;
        // sampler.setSamplingRange(env_ptr_->getOrigin(), env_ptr_->getMapSize());
        // vector<Eigen::Vector3d> preserved_samples;
        // for (int i = 0; i < 5000; ++i)
        // {
        //     Eigen::Vector3d rand_sample;
        //     sampler.uniformSamplingOnce(rand_sample);
        //     preserved_samples.push_back(rand_sample);
        // }
        // rrt_ptr_->setPreserveSamples(preserved_samples);
        // rrt_star_ptr_->setPreserveSamples(preserved_samples);
        // rrt_sharp_ptr_->setPreserveSamples(preserved_samples);

        if (run_rrt_)
        {
            bool rrt_res = rrt_ptr_->plan(start_, goal_);
            if (rrt_res)
            {
                vector<Eigen::Vector3d> final_path = rrt_ptr_->getPath();
                vis_ptr_->visualize_path(final_path, "rrt_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "rrt_final_wpts");
                vector<std::pair<double, double>> slns = rrt_ptr_->getSolutions();
                ROS_INFO_STREAM("[RRT] final path len: " << slns.back().first);
            }
        }

        if (run_rrt_star_)
        {
            bool rrt_star_res = rrt_star_ptr_->plan(start_, upper_goal);
            if (rrt_star_res)
            {
                vector<vector<Eigen::Vector3d>> routes = rrt_star_ptr_->getAllPaths();
                vis_ptr_->visualize_path_list(routes, "rrt_star_paths", visualization::yellow);
                vector<Eigen::Vector3d> final_path = rrt_star_ptr_->getPath();

                handle_finalpath(final_path);

                vis_ptr_->visualize_path(final_path, "rrt_star_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "rrt_star_final_wpts");
                vector<std::pair<double, double>> slns = rrt_star_ptr_->getSolutions();
                ROS_INFO_STREAM("[RRT*] final path len: " << slns.back().first);
                ROS_INFO_STREAM("[RRT*] final path node num: " << final_path.size());
            }
        }

        if (run_rrt_sharp_)
        {
            bool rrt_sharp_res = rrt_sharp_ptr_->plan(start_, goal_);
            if (rrt_sharp_res)
            {
                vector<Eigen::Vector3d> final_path = rrt_sharp_ptr_->getPath();
                vis_ptr_->visualize_path(final_path, "rrt_sharp_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "rrt_sharp_final_wpts");
                vector<std::pair<double, double>> slns = rrt_sharp_ptr_->getSolutions();
                ROS_INFO_STREAM("[RRT#] final path len: " << slns.back().first);
            }
        }

        if (run_brrt_)
        {
            bool brrt_res = brrt_ptr_->plan(start_, goal_);
            if (brrt_res)
            {
                vector<Eigen::Vector3d> final_path = brrt_ptr_->getPath();
                vis_ptr_->visualize_path(final_path, "brrt_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "brrt_final_wpts");
                vector<std::pair<double, double>> slns = brrt_ptr_->getSolutions();
                ROS_INFO_STREAM("[BRRT] final path len: " << slns.back().first);
            }
        }

        if (run_brrt_star_)
        {
            bool brrt_star_res = brrt_star_ptr_->plan(start_, goal_);
            if (brrt_star_res)
            {
                vector<Eigen::Vector3d> final_path = brrt_star_ptr_->getPath();
                vis_ptr_->visualize_path(final_path, "brrt_star_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "brrt_star_final_wpts");
                vector<std::pair<double, double>> slns = brrt_star_ptr_->getSolutions();
                ROS_INFO_STREAM("[BRRT*] final path len: " << slns.back().first);
            }
        }
    }
    void TesterPathFinder::executionCallback(const ros::TimerEvent &event)
    {
        if (!env_ptr_->mapValid())
        {
            ROS_INFO("no map rcved yet.");
            return;
        }
        else
        {
            execution_timer_.stop();
            return;
        }
    }

    void TesterPathFinder::handle_finalpath(vector<Eigen::Vector3d>& v)
    {
        v.clear();

        std::ifstream inFile;
        inFile.open("/home/gnij/Fast-Perching-master/bag/union/land/snap.txt");
        if (!inFile.is_open()) {
            std::cerr << "无法打开文件进行读取。\n";
            return;
        }
        double p1,p2,p3;
        while (inFile >> p1 >> p2 >> p3) 
        {
            v.emplace_back(p1,p2,p3);
        }
        cout<<"front_end_path.size(): "<<v.size()<<endl;
        inFile.close();
        return ;
    }
}


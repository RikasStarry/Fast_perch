#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <traj_opt/traj_opt.h>
#include <traj_opt/poly_traj_utils.hpp>
#include <traj_utils/PolyTraj.h>
#include <uneven_map/uneven_map.h>

#include <Eigen/Core>
#include <atomic>
#include <thread>
#include <vis_utils/vis_utils.hpp>

class PlanManager 
{
 private:
  ros::Subscriber target_sub,odom_sub;
  ros::Timer plan_timer_;
  ros::Publisher traj_pub,heartbeat_pub;

  std::shared_ptr<vis_utils::VisUtils> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;
  uneven_planner::UnevenMap::Ptr uneven_map;

  Eigen::Vector3d goal_;
  Eigen::Quaterniond goal_q;
  Eigen::Vector3d odom_pose;
  Eigen::Quaterniond odom_q;
  Eigen::Vector3d odom_lin;
  Eigen::Vector3d odom_ang;

  // NOTE just for debug
  bool debug_replan_;

  Eigen::Vector3d perching_p_, perching_v_, perching_axis_;
  double perching_theta_;

  Trajectory land_traj;
  ros::Timer exec_timer_,exec_timer2;
  int traj_id_;
  int plan_hz_;

  bool has_traj;
  std::atomic_bool target_recv,odom_recv;
 public: 
  void rcvWpsCallBack(const geometry_msgs::PoseStampedConstPtr& msgPtr);
  void debug_timer_callback(const ros::TimerEvent& event);
  void init(ros::NodeHandle& nh);
  void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, Trajectory &traj);
  void execFSMCallback(const ros::TimerEvent &e);
  void rcvOdomCallBack(const nav_msgs::Odometry::ConstPtr& msgPtr);
  void visTrajCallback(const ros::TimerEvent &e);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};
#include "/home/gnij/Fast-Perching-master/src/plan_manager/include/plan_manager.h"
#include "traj_min_jerk.hpp"

void PlanManager::rcvWpsCallBack(const geometry_msgs::PoseStampedConstPtr& msgPtr) {
  if(!uneven_map->mapReady())
    return ;
  Eigen::Matrix3d temp_m;
  Eigen::Vector3d temp_pos;
  double temp_yaw = 2 * asin(msgPtr->pose.orientation.z);
  std::cout<<"temp_yaw is :"<<temp_yaw<<std::endl;

  uneven_map->normSO2(temp_yaw);
  temp_pos << msgPtr->pose.position.x, msgPtr->pose.position.y, temp_yaw;
  uneven_map->getTerrainPos(temp_pos,temp_m,goal_);
  goal_q = temp_m;
  target_recv = true;

  geometry_msgs::Point goal_point;
  goal_point.x=goal_[0];
  goal_point.y=goal_[1];
  goal_point.z=goal_[2];
  goal_pub.publish(goal_point);
  std::cout<<"goal is "<<goal_.transpose()<<std::endl;
}

void PlanManager::rcvOdomCallBack(const nav_msgs::Odometry::ConstPtr& msgPtr) {
  odom_pose << msgPtr->pose.pose.position.x, msgPtr->pose.pose.position.y, msgPtr->pose.pose.position.z;
  odom_q.x() = msgPtr->pose.pose.orientation.x;
  odom_q.y() = msgPtr->pose.pose.orientation.y;
  odom_q.z() = msgPtr->pose.pose.orientation.z;
  odom_q.w() = msgPtr->pose.pose.orientation.w;
  odom_lin << msgPtr->twist.twist.linear.x,msgPtr->twist.twist.linear.y,msgPtr->twist.twist.linear.z;
  odom_ang << msgPtr->twist.twist.angular.x,msgPtr->twist.twist.angular.y,msgPtr->twist.twist.angular.z;
  odom_recv = true;
}

void PlanManager::debug_timer_callback(const ros::TimerEvent& event) {
  if (!(target_recv && odom_recv)) {
    return;
  }
  if(!(trajOptPtr_->init_path_bool&&trajOptPtr_->env_ptr_->mapValid()))
  {
    return;
  } 
  Eigen::MatrixXd iniState;
  iniState.setZero(3, 4);
  bool generate_new_traj_success = false;
  Trajectory traj;
  Eigen::Vector3d target_p, target_v;
  Eigen::Quaterniond land_q(1, 0, 0, 0);

  iniState.setZero();
  iniState.col(0).x() = odom_pose.x();
  iniState.col(0).y() = odom_pose.y();
  iniState.col(0).z() = odom_pose.z();
  // iniState.col(0).x() = -4.0;
  // iniState.col(0).y() = -4.0;
  // iniState.col(0).z() = 2.0;
  iniState.col(1) = odom_lin;//2,0,0
  target_p = goal_;
  target_v = Eigen::Vector3d::Zero();
  land_q = goal_q;

  std::cout << "iniState: \n" << iniState << std::endl;
  std::cout << "target_p: " << target_p.transpose() << std::endl;
  std::cout << "target_v: " << target_v.transpose() << std::endl;
  std::cout << "land_q: "
            << land_q.w() << ","
            << land_q.x() << ","
            << land_q.y() << ","
            << land_q.z() << "," << std::endl;
  if(trajOptPtr_->init_path_.size()-1 <= 1)
  {
    std::cout << "seg num error!!!"<<std::endl;
  }
  //平滑
  Eigen::MatrixXd R = land_q.toRotationMatrix();
  Eigen::Vector3d tail_q_v_ = R.col(2);
  std::cout << "slope" << tail_q_v_(2) << std::endl;
  trajOptPtr_->init_path_.back() += tail_q_v_ * trajOptPtr_->robot_l_;
  target_p += tail_q_v_ * trajOptPtr_->robot_l_;
  // Eigen::VectorXd input_time = timeAllocation(trajOptPtr_->init_path_);//  /1.4;
  // Eigen::MatrixXd input_Path(trajOptPtr_->init_path_.size(),3);
  // for(int i=0;i<input_Path.rows();i++)
  // {
  //   input_Path.row(i) = trajOptPtr_->init_path_[i];
  // }
  double temp_len_pos = 0.0;
  double total_len = 0.0;
  double piece_len = 0.8;
  
  std::vector<Eigen::Vector3d> snap_pt;

  snap_pt.emplace_back(iniState.col(0));

  for (int i=0; i<trajOptPtr_->init_path_.size()-1; i++)
  {
      double temp_seg = (trajOptPtr_->init_path_[i+1] - trajOptPtr_->init_path_[i]).norm();
      temp_len_pos += temp_seg;
      total_len += temp_seg;
      while (temp_len_pos > piece_len)
      {
          Eigen::Vector3d temp_node = trajOptPtr_->init_path_[i] + (1.0 - (temp_len_pos-piece_len) / temp_seg) * (trajOptPtr_->init_path_[i+1] - trajOptPtr_->init_path_[i]);
          snap_pt.push_back(temp_node);
          temp_len_pos -= piece_len;
      }
  }
  snap_pt.emplace_back(target_p);
  Eigen::VectorXd input_time = timeAllocation(snap_pt);//  /1.4;
  // cout<<"snap_pt: "<<endl;
  // for(int i=0;i<snap_pt.size();i++)
  // {
  //   cout<<snap_pt[i].transpose()<<'\n';
  //   if(i>=1)
  //     cout<<"dist:"<<(snap_pt[i]-snap_pt[i-1]).norm()<<'\n';
  // }
  // for(int i=0;i<input_time.rows();i++)
  //   cout<<input_time[i]<<'\n';

  Eigen::MatrixXd input_Path(3,snap_pt.size());
  for(int i=0;i<input_Path.cols();i++)
  {
    input_Path.col(i) = snap_pt[i];
  }
  
  Eigen::MatrixXd iS = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd fS = Eigen::MatrixXd::Zero(3, 3);
  iS.col(0) << input_Path.leftCols<1>();
  fS.col(0) << input_Path.rightCols<1>();
  min_jerk::JerkOpt jerkOpt;
  jerkOpt.reset(iS, fS, input_Path.cols() - 1);
  jerkOpt.generate(input_Path.block(0, 1, 3, input_Path.cols() - 2), input_time);

  std::vector<double> snap_ts(input_time.rows());
  for(int i=0;i<snap_ts.size();i++)
    snap_ts[i] = input_time[i];
  trajOptPtr_->snap_traj = Trajectory(snap_ts,jerkOpt.getCoefficientMats());
  // double temp=0;
  // for(int i=0;i<input_time.size()-1;i++)
  // {
  //   std::cout<<"Time is "<<input_time[i]<<std::endl;
  //   std::cout<<"node is "<<trajOptPtr_->snap_traj.getPos(temp)<<std::endl;
  //   std::cout<<"v is "<<(trajOptPtr_->snap_traj.getPos(temp+input_time[i])-trajOptPtr_->snap_traj.getPos(temp)).norm()/input_time[i]<<std::endl;
  //   temp+=input_time[i];
  // }
  trajOptPtr_->has_snap_traj = true;

  int N = 10;

  // std::ofstream outFile;
  // outFile.open("/home/gnij/Fast-Perching-master/bag/real/kino.txt",std::ios_base::app);
  // if (!outFile.is_open()) {
  //     std::cerr << "无法打开文件进行写入。\n";
  //     return ;
  // }
  // for (int i = 0; i <= 20; ++i) {
  //   double total_t = trajOptPtr_->snap_traj.getTotalDuration();
  //   Eigen::Vector3d temp_pos = trajOptPtr_->snap_traj.getPos(double(i)/(20) * total_t);
  //   outFile << temp_pos.x() << ' ' << temp_pos.y() << ' ' << temp_pos.z() << '\n';
  // }
  // outFile.close();


  //trajOptPtr_->init_path_.size()-1;
  //int N = trajOptPtr_->snap_traj.getTrajLenth() / trajOptPtr_->piece_len + 1;

  // double temp_len_pos = 0.0;
  // double total_len = 0.0;
  // double piece_len = 0.8;
  
  // for (int i=0; i<trajOptPtr_->init_path_.size()-1; i++)
  // {
  //     double temp_seg = (trajOptPtr_->init_path_[i+1] - trajOptPtr_->init_path_[i]).norm();
  //     temp_len_pos += temp_seg;
  //     total_len += temp_seg;
  //     if (temp_len_pos > piece_len)
  //     {
  //         Eigen::Vector3d temp_node = trajOptPtr_->init_path_[i] + (1.0 - (temp_len_pos-piece_len) / temp_seg) * (trajOptPtr_->init_path_[i+1] - trajOptPtr_->init_path_[i]);
  //         trajOptPtr_->inner_xy_node.push_back(temp_node);
  //         temp_len_pos -= piece_len;
  //     }
  // }
  // visPtr_->visualize_pointcloud(trajOptPtr_->inner_xy_node,"rrt_traj_wpts");
  // trajOptPtr_->total_time = total_len / trajOptPtr_->vmax_ * 2.0;
  // int N = trajOptPtr_->inner_xy_node.size()+ 1;

  generate_new_traj_success = trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, N, traj);
  if (generate_new_traj_success) {
    visPtr_->visualize_traj(traj, "traj");
    land_traj = traj;
    //land_traj = trajOptPtr_->snap_traj;
    has_traj = true;
    //Eigen::Vector3d tail_pos = land_traj.getPos(land_traj.getTotalDuration());
    //Eigen::Vector3d tail_vel = land_traj.getVel(land_traj.getTotalDuration());
    //visPtr_->visualize_arrow(tail_pos, tail_pos + 0.5 * land_traj.getVel(land_traj.getTotalDuration()), "tail_vel");
    //visPtr_->visualize_traj(land_traj, "traj");
    traj_utils::PolyTraj poly_msg;
    polyTraj2ROSMsg(poly_msg,land_traj);
    traj_pub.publish(poly_msg);
  }
  if (!generate_new_traj_success) {
    std::cout << "Generate new traj failed ! " <<std::endl;
    target_recv = false;
    return;
    // assert(false);
  }

  // NOTE run vis
  // hopf fiberation
  auto v2q = [](const Eigen::Vector3d& v, Eigen::Quaterniond& q) -> bool {
    double a = v.x();
    double b = v.y();
    double c = v.z();
    if (c == -1) {
      return false;
    }
    double d = 1.0 / sqrt(2.0 * (1 + c));
    q.w() = (1 + c) * d;
    q.x() = -b * d;
    q.y() = a * d;
    q.z() = 0;
    return true;
  };

  auto f_DN = [](const Eigen::Vector3d& x) {
    double x_norm_2 = x.squaredNorm();
    return (Eigen::MatrixXd::Identity(3, 3) - x * x.transpose() / x_norm_2) / sqrt(x_norm_2);
  };
  // auto f_D2N = [](const Eigen::Vector3d& x, const Eigen::Vector3d& y) {
  //   double x_norm_2 = x.squaredNorm();
  //   double x_norm_3 = x_norm_2 * x.norm();
  //   Eigen::MatrixXd A = (3 * x * x.transpose() / x_norm_2 - Eigen::MatrixXd::Identity(3, 3));
  //   return (A * y * x.transpose() - x * y.transpose() - x.dot(y) * Eigen::MatrixXd::Identity(3, 3)) / x_norm_3;
  // };

  nav_msgs::Odometry msg;
  msg.header.frame_id = "world";
  double dt = 0.001;
  Eigen::Quaterniond q_last;
  double max_omega = 0;
  for (double t = 0; t <= land_traj.getTotalDuration(); t += dt) {
    ros::Duration(dt).sleep();
    // drone
    Eigen::Vector3d p = land_traj.getPos(t);
    Eigen::Vector3d a = land_traj.getAcc(t);
    Eigen::Vector3d j = land_traj.getJer(t);
    Eigen::Vector3d g(0, 0, -9.8);
    Eigen::Vector3d thrust = a - g;

    // std::cout << p.x() << " , " << p.z() << " , ";

    Eigen::Vector3d zb = thrust.normalized();
    {
      // double a = zb.x();
      // double b = zb.y();
      // double c = zb.z();
      Eigen::Vector3d zb_dot = f_DN(thrust) * j;
      double omega12 = zb_dot.norm();
      // if (omega12 > 3.1) {
      //   std::cout << "omega: " << omega12 << "rad/s  t: " << t << std::endl;
      // }
      if (omega12 > max_omega) {
        max_omega = omega12;
      }
      // double a_dot = zb_dot.x();
      // double b_dot = zb_dot.y();
      // double omega3 = (b * a_dot - a * b_dot) / (1 + c);
      // std::cout << "jer: " << j.transpose() << std::endl;
      // std::cout << "omega12: " << zb_dot.norm() << std::endl;
      // std::cout << "omega3: " << omega3 << std::endl;
      // std::cout << thrust.x() << " , " << thrust.z() << " , ";
      // double omega2 = zb_dot.x() - zb.x() * zb_dot.z() / (zb.z() + 1);
      // std::cout << omega2 << std::endl;
      // std::cout << zb_dot.norm() << std::endl;
    }

    Eigen::Quaterniond q;
    bool no_singlarity = v2q(zb, q);
    Eigen::MatrixXd R_dot = (q.toRotationMatrix() - q_last.toRotationMatrix()) / dt;
    Eigen::MatrixXd omega_M = q.toRotationMatrix().transpose() * R_dot;
    // std::cout << "omega_M: \n" << omega_M << std::endl;
    Eigen::Vector3d omega_real;
    omega_real.x() = -omega_M(1, 2);
    omega_real.y() = omega_M(0, 2);
    omega_real.z() = -omega_M(0, 1);
    // std::cout << "omega_real: " << omega_real.transpose() << std::endl;
    q_last = q;
    if (!no_singlarity) {//无奇点
      std::cout << "Occur singlarity!" << std::endl;
    }

    if (trajOptPtr_->check_collilsion(p, a, target_p)) {
      std::cout << "collide!  t: " << t << std::endl;
    }
    // // TODO replan
    // if (debug_replan_ && t > 1.0 / plan_hz_ && traj.getTotalDuration() > 0.5) {
    //   // ros::Duration(3.0).sleep();

    //   iniState.col(0) = traj.getPos(t);
    //   iniState.col(1) = traj.getVel(t);
    //   iniState.col(2) = traj.getAcc(t);
    //   iniState.col(3) = traj.getJer(t);
    //   std::cout << "iniState: \n"
    //             << iniState << std::endl;
    //   //从当前时刻和位置重新规划
    //   trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj, t);
    //   visPtr_->visualize_traj(traj, "traj");
    //   t = 0;
    //   std::cout << "max omega: " << max_omega << std::endl;
    // }
  }

  std::cout << "tailV: " << land_traj.getVel(land_traj.getTotalDuration()).transpose() << std::endl;
  std::cout << "max thrust: " << land_traj.getMaxThrust() << std::endl;
  std::cout << "max omega: " << max_omega << std::endl;

  std::cout << "maxVel: " << land_traj.getMaxVelRate() << std::endl;
  std::cout << "maxAcc: " << land_traj.getMaxAccRate() << std::endl;
  std::cout << "total time: " << land_traj.getTotalDuration() << std::endl;
  std::cout << "total lenth: " << land_traj.getTrajLenth() << std::endl;
  std::cout << "endVel: " << land_traj.getVel(land_traj.getTotalDuration()) << std::endl;
  std::cout << "finall traj len is " << land_traj.getTrajLenth()<< std::endl;
  std::cout << "final traj time is " << land_traj.getTotalDuration()<< std::endl;

  target_recv = false;
}

void PlanManager::init(ros::NodeHandle& nh) {
  // set parameters of planning
  nh.getParam("replan", debug_replan_);
  nh.getParam("plan_hz", plan_hz_);

  visPtr_ = std::make_shared<vis_utils::VisUtils>(nh);
  trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh);
  uneven_map.reset(new uneven_planner::UnevenMap);
  uneven_map->init(nh);
  pfPtr_ = std::make_shared<PF::TesterPathFinder>(nh,trajOptPtr_->env_ptr_,uneven_map);

  //ros timer之间存在阻塞
  plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz_), &PlanManager::debug_timer_callback, this);
  traj_pub = nh.advertise<traj_utils::PolyTraj>("/land/trajectory", 10);
  heartbeat_pub = nh.advertise<std_msgs::Empty>("/land/heartbeat", 10);

  odom_sub = nh.subscribe<nav_msgs::Odometry>("/gazebo_odom", 1, &PlanManager::rcvOdomCallBack, this);
  target_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &PlanManager::rcvWpsCallBack, this);

  exec_timer_ = nh.createTimer(ros::Duration(0.01), &PlanManager::execFSMCallback, this);
  exec_timer2 = nh.createTimer(ros::Duration(0.5), &PlanManager::visTrajCallback, this);

  goal_pub = nh.advertise<geometry_msgs::Point>("uneven_goal", 10);

  ROS_WARN("Planning node initialized!");

  debug_replan_ = false;
  traj_id_ = 0;
  has_traj = false;
  target_recv = ATOMIC_VAR_INIT(false);
  odom_recv = ATOMIC_VAR_INIT(false);
}


void PlanManager::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, Trajectory &traj)
{
    Eigen::VectorXd durs = traj.getDurations();
    int piece_num = traj.getPieceNum();

    poly_msg.drone_id = 0;
    poly_msg.traj_id = traj_id_;
    poly_msg.start_time = ros::Time::now();//
    poly_msg.order = 7; 
    poly_msg.duration.resize(piece_num);
    poly_msg.coef_x.resize(8 * piece_num);
    poly_msg.coef_y.resize(8 * piece_num);
    poly_msg.coef_z.resize(8 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      poly_msg.duration[i] = durs(i);

      CoefficientMat cMat = traj[i].getCoeffMat();
      int i8 = i * 8;
      for (int j = 0; j < 8; j++)
      {
        poly_msg.coef_x[i8 + j] = cMat(0, j);
        poly_msg.coef_y[i8 + j] = cMat(1, j);
        poly_msg.coef_z[i8 + j] = cMat(2, j);
      }
    }
}

void PlanManager::execFSMCallback(const ros::TimerEvent &e)
{
  std_msgs::Empty heartbeat_msg;
  heartbeat_pub.publish(heartbeat_msg);
}

void PlanManager::visTrajCallback(const ros::TimerEvent &e)
{
  if(trajOptPtr_->has_snap_traj)
  {
    visPtr_->visualize_traj(trajOptPtr_->snap_traj, "snap_traj");

  }
    
  if(!has_traj)
    return ;
  visPtr_->visualize_traj(land_traj, "traj");
  // Eigen::Vector3d tail_pos = land_traj.getPos(land_traj.getTotalDuration());
  // Eigen::Vector3d tail_vel = land_traj.getVel(land_traj.getTotalDuration());
  // visPtr_->visualize_arrow(tail_pos, tail_pos + 0.5 * land_traj.getVel(land_traj.getTotalDuration()), "tail_vel");
}

int PlanManager::Factorial(int x) {
    int fac = 1;
    for (int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

Eigen::VectorXd PlanManager::timeAllocation(const std::vector<Eigen::Vector3d>& Path) {
    cout<<"trajOptPtr_->vmax_:"<<trajOptPtr_->vmax_<<endl;
    Eigen::VectorXd times(Path.size() - 1);
    const double t = trajOptPtr_->vmax_ / trajOptPtr_->amax_;
    const double dist_threshold_1 = trajOptPtr_->amax_ * t * t;

    double segment_t;
    for (unsigned int i = 1; i < Path.size(); ++i) {
        double delta_dist = (Path[i] - Path[i - 1]).norm();
        if (delta_dist > dist_threshold_1) {
            segment_t = t * 2 + (delta_dist - dist_threshold_1) / trajOptPtr_->vmax_;
        } else {
            segment_t = std::sqrt(delta_dist / trajOptPtr_->amax_);//delta_dist/trajOptPtr_->vmax_;
        }
        times[i - 1] = segment_t;

        // if(i==1)
        // {
        //   times[i - 1] = delta_dist/trajOptPtr_->vmax_*1.2;//0.5
        // }
        // else if(i==2)
        // {
        //   times[i - 1] = delta_dist/trajOptPtr_->vmax_*1.2;//0.5
        // }
        // else if(i==Path.size()-2)
        // {
        //   times[i - 1] = delta_dist/trajOptPtr_->vmax_*0.8;//0.45
        // }
        // else if(i==Path.size()-1)
        // {
        //   times[i - 1] = delta_dist/trajOptPtr_->vmax_*0.8;//0.55
        // }
        // else{
        //   times[i - 1] = delta_dist/trajOptPtr_->vmax_*1.0;
        // }
        
    }
    return times;
}




#pragma once
#include <ros/ros.h>

#include <chrono>
#include <thread>
#include <vis_utils/vis_utils.hpp>
#include <nav_msgs/Path.h>

#include "minco.hpp"
#include "occ_grid/occ_map.h"

namespace traj_opt {
class SFC
{
  public:
    std::vector<Eigen::MatrixXd> sfc_A;
    std::vector<Eigen::VectorXd>sfc_b;
    const Eigen::MatrixXd& returnA(int index)
    {
      return sfc_A[index];
    }
    const Eigen::VectorXd& returnb(int index)
    {
      return sfc_b[index];
    }
    void push_backA(Eigen::MatrixXd& A)
    {
      sfc_A.push_back(A);
    }
    void push_backb(Eigen::VectorXd& b)
    {
      sfc_b.push_back(b);
    }
};
class TrajOpt {
 public:
  ros::NodeHandle nh_;
  std::shared_ptr<vis_utils::VisUtils> visPtr_;
  bool pause_debug_ = false;
  // # pieces and # key points
  int N_, K_, dim_t_, dim_p_;
  // weight for time regularization term
  double rhoT_, rhoVt_;
  // collision avoiding and dynamics paramters
  double vmax_, amax_;
  double rhoP_, rhoV_, rhoA_;
  double rhoThrust_, rhoOmega_;
  double rhoPerchingCollision_;
  // landing parameters
  double v_plus_, robot_l_, robot_r_, platform_r_;
  // SE3 dynamic limitation parameters
  double thrust_max_, thrust_min_;
  double omega_max_, omega_yaw_max_;
  // MINCO Optimizer
  minco::MINCO_S4_Uniform mincoOpt_;
  Eigen::MatrixXd initS_;
  // duration of each piece of the trajectory
  Eigen::VectorXd t_;
  double* x_;
  //occ_map
  env::OccMap::Ptr env_ptr_;
  //init_path
  ros::Subscriber path_sub;
  vector<Eigen::Vector3d> init_path_;
  bool init_path_bool=false;
  Trajectory snap_traj;
  bool has_snap_traj=false;
  std::vector<Eigen::Vector3d> inner_xy_node;
  double piece_len;
  //double total_time;
  Trajectory final_traj;

  ros::Publisher ellipsoid_pub;

  SFC sfc;
  std::ofstream costRecorder;

 public:
  TrajOpt(ros::NodeHandle& nh);
  ~TrajOpt() {}

  int optimize(const double& delta = 1e-4);
  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::Vector3d& car_p,
                     const Eigen::Vector3d& car_v,
                     const Eigen::Quaterniond& land_q,
                     const int& N,
                     Trajectory& traj, 
                     const double& t_replan = -1.0);

  bool feasibleCheck(Trajectory& traj);

  void addTimeIntPenalty(double& cost);
  void addSmoothPenalty(double& cost);
  bool grad_cost_v(const Eigen::Vector3d& v,
                   Eigen::Vector3d& gradv,
                   double& costv);
  bool grad_cost_thrust(const Eigen::Vector3d& a,
                        Eigen::Vector3d& grada,
                        double& costa);
  bool grad_cost_omega(const Eigen::Vector3d& a,
                       const Eigen::Vector3d& j,
                       Eigen::Vector3d& grada,
                       Eigen::Vector3d& gradj,
                       double& cost);
  bool grad_cost_omega_yaw(const Eigen::Vector3d& a,
                           const Eigen::Vector3d& j,
                           Eigen::Vector3d& grada,
                           Eigen::Vector3d& gradj,
                           double& cost);
  bool grad_cost_floor(const Eigen::Vector3d& p,
                       Eigen::Vector3d& gradp,
                       double& costp,int i);
  bool grad_cost_perching_collision(const Eigen::Vector3d& pos,
                                    const Eigen::Vector3d& acc,
                                    const Eigen::Vector3d& car_p,
                                    Eigen::Vector3d& gradp,
                                    Eigen::Vector3d& grada,
                                    Eigen::Vector3d& grad_car_p,
                                    double& cost);
  bool check_collilsion(const Eigen::Vector3d& pos,
                        const Eigen::Vector3d& acc,
                        const Eigen::Vector3d& car_p);
  void pathCallback(const nav_msgs::Path::ConstPtr& msg);
  void parseFile(std::string filename);
};

void TrajOpt::addSmoothPenalty(double& cost) {
  Eigen::Vector3d pos1, vel1,  pos2, vel2, pos3, vel3;
  Eigen::Vector3d grad_p1,grad_p2,grad_p3;
  Eigen::Matrix<double, 8, 1> beta0_1, beta1_1,beta0_2, beta1_2,beta0_3, beta1_3;
  double s1, s2, s3, s4, s5, s6, s7;
  double step;
  double rhoSmooth = 50000;
  int innerLoop = K_ + 1;
  step = mincoOpt_.t(1) / K_;

  const auto& c = mincoOpt_.c.block<8, 3>(N_-1 * 8, 0);

  for (int j = 0; j < innerLoop-2; ++j) {

    s1 = j * step;
    s2 = s1 * s1;
    s3 = s2 * s1;
    s4 = s2 * s2;
    s5 = s4 * s1;
    s6 = s4 * s2;
    s7 = s4 * s3;
    beta0_1 << 1.0, s1, s2, s3, s4, s5, s6, s7;
    beta1_1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
    pos1 = c.transpose() * beta0_1;
    vel1 = c.transpose() * beta1_1;

    s1 += step;
    s2 = s1 * s1;
    s3 = s2 * s1;
    s4 = s2 * s2;
    s5 = s4 * s1;
    s6 = s4 * s2;
    s7 = s4 * s3;
    beta0_2 << 1.0, s1, s2, s3, s4, s5, s6, s7;
    beta1_2 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
    pos2 = c.transpose() * beta0_2;
    vel2 = c.transpose() * beta1_2;

    s1 += step;
    s2 = s1 * s1;
    s3 = s2 * s1;
    s4 = s2 * s2;
    s5 = s4 * s1;
    s6 = s4 * s2;
    s7 = s4 * s3;
    beta0_3 << 1.0, s1, s2, s3, s4, s5, s6, s7;
    beta1_3 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
    pos3 = c.transpose() * beta0_3;
    vel3 = c.transpose() * beta1_3;

    cost += rhoSmooth * (pos1 + pos2 - 2 * pos2).squaredNorm();
    grad_p1 = rhoSmooth * (pos1 + pos2 - 2 * pos2) * 2;
    grad_p2 = rhoSmooth * (pos1 + pos2 - 2 * pos2) * -4;
    grad_p3 = rhoSmooth * (pos1 + pos2 - 2 * pos2) * 2;
    mincoOpt_.gdC.block<8,3>(N_-1 * 8, 0) += beta0_1*grad_p1.transpose() + beta0_1*grad_p2.transpose() + beta0_3*grad_p3.transpose();
    
    double gdt1 = double(j)/K_ * grad_p1.transpose() * vel1 ;
    double gdt2 = double(j+1)/K_ * grad_p2.transpose() * vel2;
    double gdt3 = double(j+2)/K_ * grad_p3.transpose() * vel3;
    mincoOpt_.gdT += gdt1+gdt2+gdt3;
  }
}
void TrajOpt::parseFile(std::string filename)
{
  std::ifstream file(filename);
  if (!file.is_open()) {
      std::cerr << "无法打开文件: " << filename << std::endl;
      return;
  }

  // 循环解析每一组数据
  while (true) {
      int rows; // 矩阵 A 的行数
      file >> rows;

      // 如果读取失败，说明到达文件末尾
      if (file.eof()) break;

      // 忽略剩余的换行符
      file.ignore();

      // 假设矩阵 A 的列数固定为 3
      int cols = 3;

      // 初始化矩阵 A 和向量 b
      Eigen::MatrixXd A(rows, cols);
      Eigen::VectorXd b(rows);

      // 读取矩阵 A 的内容
      for (int r = 0; r < rows; ++r) {
          std::string line;
          std::getline(file, line); // 获取一行
          std::stringstream ss(line);
          for (int c = 0; c < cols; ++c) {
              ss >> A(r, c); // 逐个读取元素
          }
      }
      file.ignore(2);
      // 读取向量 b 的内容
      std::string b_line;
      std::getline(file, b_line); // 获取 b 的一行
      std::stringstream b_ss(b_line);
      for (int i = 0; i < rows; ++i) {
          b_ss >> b(i); // 逐个读取 b 的元素
      }

      // 将解析结果存入约束列表
      sfc.push_backA(A);
      sfc.push_backb(b);
  }
  // 关闭文件
  file.close();
  // for (size_t i = 0; i < sfc.sfc_A.size(); ++i) {
  //     std::cout << "第 " << i  << " 组数据:\n";
  //     std::cout << "矩阵 A:\n" << sfc.returnA(i) << "\n";
  //     std::cout << "向量 b:\n" << sfc.returnb(i) << "\n";
  // }  
}  // parseFile

}  // namespace traj_opt
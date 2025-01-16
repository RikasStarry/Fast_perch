#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
#include <traj_opt/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

using namespace Eigen;

ros::Publisher pos_cmd_pub;

quadrotor_msgs::PositionCommand cmd;
// double pos_gain[3] = {0, 0, 0};
// double vel_gain[3] = {0, 0, 0};

#define FLIP_YAW_AT_END 0
#define TURN_YAW_TO_CENTER_AT_END 0

bool receive_traj_ = false;
boost::shared_ptr<Trajectory> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
ros::Time heartbeat_time_(0);
Eigen::Vector3d last_pos_;
Eigen::Vector3d last_vel_;
Eigen::Vector3d last_acc_;
Eigen::Vector3d last_jer_;

// yaw control
double last_yaw_, last_yawdot_, slowly_flip_yaw_target_, slowly_turn_to_center_target_;
double time_forward_;

void heartbeatCallback(std_msgs::EmptyPtr msg)
{
   heartbeat_time_ = ros::Time::now();
}

void polyTrajCallback(traj_utils::PolyTrajPtr msg)
{
  if (msg->order != 7)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 7 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i8 = i * 8;
    cMats[i].row(0) << msg->coef_x[i8 + 0], msg->coef_x[i8 + 1], msg->coef_x[i8 + 2],
                       msg->coef_x[i8 + 3], msg->coef_x[i8 + 4], msg->coef_x[i8 + 5],
                       msg->coef_x[i8 + 6], msg->coef_x[i8 + 7];
    cMats[i].row(1) << msg->coef_y[i8 + 0], msg->coef_y[i8 + 1], msg->coef_y[i8 + 2],
                       msg->coef_y[i8 + 3], msg->coef_y[i8 + 4], msg->coef_y[i8 + 5],
                       msg->coef_y[i8 + 6], msg->coef_y[i8 + 7];
    cMats[i].row(2) << msg->coef_z[i8 + 0], msg->coef_z[i8 + 1], msg->coef_z[i8 + 2],
                       msg->coef_z[i8 + 3], msg->coef_z[i8 + 4], msg->coef_z[i8 + 5],
                       msg->coef_z[i8 + 6], msg->coef_z[i8 + 7];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new Trajectory(dura, cMats));

  start_time_ = ros::Time::now();
  traj_duration_ = traj_->getTotalDuration();
  traj_id_ = msg->traj_id;

  receive_traj_ = true;
}

// std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
// {
//   constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
//   constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;
//   std::pair<double, double> yaw_yawdot(0, 0);

//   Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
//                             ? traj_->getPos(t_cur + time_forward_) - pos
//                             : traj_->getPos(traj_duration_) - pos;
//   double yaw_temp = dir.norm() > 0.1
//                         ? atan2(dir(1), dir(0))
//                         : last_yaw_;

//   double yawdot = 0;
//   double d_yaw = yaw_temp - last_yaw_;
//   if (d_yaw >= M_PI)
//   {
//     d_yaw -= 2 * M_PI;
//   }
//   if (d_yaw <= -M_PI)
//   {
//     d_yaw += 2 * M_PI;
//   }

//   const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
//   const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
//   double d_yaw_max;
//   if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
//   {
//     // yawdot = last_yawdot_ + dt * YDDM;
//     d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
//   }
//   else
//   {
//     // yawdot = YDM;
//     double t1 = (YDM - last_yawdot_) / YDDM;
//     d_yaw_max = ((dt - t1) + dt) * (YDM - last_yawdot_) / 2.0;
//   }

//   if (fabs(d_yaw) > fabs(d_yaw_max))
//   {
//     d_yaw = d_yaw_max;
//   }
//   yawdot = d_yaw / dt;

//   double yaw = last_yaw_ + d_yaw;
//   if (yaw > M_PI)
//     yaw -= 2 * M_PI;
//   if (yaw < -M_PI)
//     yaw += 2 * M_PI;
//   yaw_yawdot.first = yaw;
//   yaw_yawdot.second = yawdot;

//   last_yaw_ = yaw_yawdot.first;
//   last_yawdot_ = yaw_yawdot.second;

//   yaw_yawdot.second = yaw_temp;

//   return yaw_yawdot;
// }
std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_          \
                            ? traj_->getPos(t_cur + time_forward_) - pos \
                            : traj_->getPos(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.0 ? atan2(dir(1), dir(0)) : last_yaw_;
  //std::cout<<"yaw_temp is :"<<yaw_temp<<std::endl;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * dt;
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yawdot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yawdot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd)
{

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  if(!receive_traj_)
  {
    cmd.trajectory_id = traj_id_ + 1;
  }
  else
  {
    cmd.trajectory_id = traj_id_;
  }


  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.jerk.x = j(0);
  cmd.jerk.y = j(1);
  cmd.jerk.z = j(2);
  cmd.yaw = y;
  //std::cout<<"yaw is :"<<y<<std::endl;
  cmd.yaw_dot = yd;
  pos_cmd_pub.publish(cmd);

  last_pos_ = p;
  last_vel_ = v;
  last_acc_ = a;
  last_jer_ = j;
}

void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ and have heartbeat */
  if (heartbeat_time_.toSec() <= 1e-5)
  {
    // ROS_ERROR_ONCE("[traj_server] No heartbeat from the planner received");
    return;
  }
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  //std::cout<<"time_now - heartbeat_time_:"<<(time_now - heartbeat_time_).toSec()<<std::endl;
  // if ((time_now - heartbeat_time_).toSec() > 0.5)
  // {
  //   ROS_ERROR("[traj_server] Lost heartbeat from the planner, is it dead?");
  // }
  //std::cout<<"time_now - start_time_:"<<(time_now - start_time_).toSec()<<std::endl;
  double t_cur = (time_now - start_time_).toSec();
  if (t_cur > traj_duration_)
  {
    ROS_WARN("[traj_server] Traj has ended !");

    receive_traj_ = false;
    //终点非静止，以一定速度冲出  
    publish_cmd(last_pos_, last_vel_, last_acc_, last_jer_, last_yaw_, 0);
  }

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
  static bool finished = false;
#endif
  if (t_cur <= traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jer = traj_->getJer(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, (time_now - time_last).toSec());
    /*** calculate yaw ***/

    time_last = time_now;
    last_yaw_ = yaw_yawdot.first;
    last_pos_ = pos;

    slowly_flip_yaw_target_ = yaw_yawdot.first + M_PI;
    if (slowly_flip_yaw_target_ > M_PI)
      slowly_flip_yaw_target_ -= 2 * M_PI;
    if (slowly_flip_yaw_target_ < -M_PI)
      slowly_flip_yaw_target_ += 2 * M_PI;
    constexpr double CENTER[2] = {0.0, 0.0};
    slowly_turn_to_center_target_ = atan2(CENTER[1] - pos(1), CENTER[0] - pos(0));

    // publish
    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
    finished = false;
#endif
  }

#if FLIP_YAW_AT_END
  else if (t_cur >= traj_duration_)
  {
    if (finished)
      return;

    /* hover when finished traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();

    if (slowly_flip_yaw_target_ > 0)
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ >= slowly_flip_yaw_target_)
      {
        finished = true;
      }
    }
    else
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ <= slowly_flip_yaw_target_)
      {
        finished = true;
      }
    }

    yaw_yawdot.first = last_yaw_;
    time_last = time_now;

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
#endif

#if TURN_YAW_TO_CENTER_AT_END
  else if (t_cur >= traj_duration_)
  {
    if (finished)
      return;

    /* hover when finished traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();

    double d_yaw = last_yaw_ - slowly_turn_to_center_target_;
    if (d_yaw >= M_PI)
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ > M_PI)
        last_yaw_ -= 2 * M_PI;
    }
    else if (d_yaw <= -M_PI)
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ < -M_PI)
        last_yaw_ += 2 * M_PI;
    }
    else if (d_yaw >= 0)
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ <= slowly_turn_to_center_target_)
        finished = true;
    }
    else
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ >= slowly_turn_to_center_target_)
        finished = true;
    }

    yaw_yawdot.first = last_yaw_;
    time_last = time_now;

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
#endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_sub = nh.subscribe("/land/trajectory", 10, polyTrajCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("/land/heartbeat", 10, heartbeatCallback);

  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  nh.param("time_forward", time_forward_, 0.0);
  last_yaw_ = 0.0;
  last_yawdot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server]: ready.");

  ros::spin();

  return 0;
}
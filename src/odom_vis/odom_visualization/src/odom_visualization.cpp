#include <string.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/StdVector>
#include <iostream>

#include "armadillo"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "pose_utils.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

using namespace arma;
using namespace std;

static string mesh_resource;
static double scale;
bool cross_config = false;
bool origin = false;
bool isOriginSet = false;
colvec poseOrigin(6);
//ros::Publisher posePub;
//ros::Publisher pathPub;
//ros::Publisher trajPub;
ros::Publisher meshPub;
//geometry_msgs::PoseStamped poseROS;
//nav_msgs::Path pathROS;
//visualization_msgs::Marker trajROS;
visualization_msgs::Marker meshROS;
string _frame_id;

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  if (msg->header.frame_id == string("null"))
    return;
  colvec pose(6);
  colvec q(4);
  colvec vel(3);

  pose(0) = msg->pose.pose.position.x;
  pose(1) = msg->pose.pose.position.y;
  pose(2) = msg->pose.pose.position.z;
  q(0) = msg->pose.pose.orientation.w;
  q(1) = msg->pose.pose.orientation.x;
  q(2) = msg->pose.pose.orientation.y;
  q(3) = msg->pose.pose.orientation.z;
  pose.rows(3, 5) = R_to_ypr(quaternion_to_R(q));
  vel(0) = msg->twist.twist.linear.x;
  vel(1) = msg->twist.twist.linear.y;
  vel(2) = msg->twist.twist.linear.z;

  // NOTE fov
  //Eigen::Vector3d fov_p(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  //Eigen::Quaterniond fov_q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

  if (origin && !isOriginSet) {
    isOriginSet = true;
    poseOrigin = pose;
  }
  if (origin) {
    vel = trans(ypr_to_R(pose.rows(3, 5))) * vel;
    pose = pose_update(pose_inverse(poseOrigin), pose);
    vel = ypr_to_R(pose.rows(3, 5)) * vel;
  }

  // Mesh model
  meshROS.header.frame_id = _frame_id;
  meshROS.header.stamp = msg->header.stamp;
  meshROS.ns = "mesh";
  meshROS.id = 0;
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::Marker::ADD;
  meshROS.mesh_use_embedded_materials = true;
  // meshROS.pose.position.x = msg->pose.pose.position.x;
  // meshROS.pose.position.y = msg->pose.pose.position.y;
  // meshROS.pose.position.z = msg->pose.pose.position.z;
  // q(0) = msg->pose.pose.orientation.w;
  // q(1) = msg->pose.pose.orientation.x;
  // q(2) = msg->pose.pose.orientation.y;
  // q(3) = msg->pose.pose.orientation.z;
  meshROS.pose.position.x = pose(0);
  meshROS.pose.position.y = pose(1);
  meshROS.pose.position.z = pose(2);

  if (cross_config) {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0) += 45.0 * PI / 180.0;
    q = R_to_quaternion(ypr_to_R(ypr));
  }

  meshROS.pose.orientation.w = q(0);
  meshROS.pose.orientation.x = q(1);
  meshROS.pose.orientation.y = q(2);
  meshROS.pose.orientation.z = q(3);
  meshROS.scale.x = scale;
  meshROS.scale.y = scale;
  meshROS.scale.z = scale;
  // meshROS.color.a = color_a;
  // meshROS.color.r = color_r;
  // meshROS.color.g = color_g;
  // meshROS.color.b = color_b;
  meshROS.color.a = 0;
  meshROS.color.r = 0;
  meshROS.color.g = 0;
  meshROS.color.b = 0;
  meshROS.mesh_resource = mesh_resource;
  meshPub.publish(meshROS);

  // // Pose
  // poseROS.header = msg->header;
  // poseROS.header.stamp = msg->header.stamp;
  // poseROS.header.frame_id = string("world");
  // poseROS.pose.position.x = pose(0);
  // poseROS.pose.position.y = pose(1);
  // poseROS.pose.position.z = pose(2);
  // q = R_to_quaternion(ypr_to_R(pose.rows(3, 5)));
  // poseROS.pose.orientation.w = q(0);
  // poseROS.pose.orientation.x = q(1);
  // poseROS.pose.orientation.y = q(2);
  // poseROS.pose.orientation.z = q(3);
  // posePub.publish(poseROS);

  // // Path
  // static ros::Time prevt = msg->header.stamp;
  // if ((msg->header.stamp - prevt).toSec() > 0.1) {
  //   prevt = msg->header.stamp;
  //   pathROS.header = poseROS.header;
  //   pathROS.poses.push_back(poseROS);
  //   pathPub.publish(pathROS);
  // }

  // // Covariance color
  // double r = 1;
  // double g = 1;
  // double b = 1;
  
  // // Color Coded Trajectory
  // static colvec ppose = pose;
  // static ros::Time pt = msg->header.stamp;
  // ros::Time t = msg->header.stamp;
  // if ((t - pt).toSec() > 0.5) {
  //   trajROS.header.frame_id = string("world");
  //   trajROS.header.stamp = ros::Time::now();
  //   trajROS.ns = string("trajectory");
  //   trajROS.type = visualization_msgs::Marker::LINE_LIST;
  //   trajROS.action = visualization_msgs::Marker::ADD;
  //   trajROS.pose.position.x = 0;
  //   trajROS.pose.position.y = 0;
  //   trajROS.pose.position.z = 0;
  //   trajROS.pose.orientation.w = 1;
  //   trajROS.pose.orientation.x = 0;
  //   trajROS.pose.orientation.y = 0;
  //   trajROS.pose.orientation.z = 0;
  //   trajROS.scale.x = 0.1;
  //   trajROS.scale.y = 0;
  //   trajROS.scale.z = 0;
  //   trajROS.color.r = 0.0;
  //   trajROS.color.g = 1.0;
  //   trajROS.color.b = 0.0;
  //   trajROS.color.a = 0.8;
  //   geometry_msgs::Point p;
  //   p.x = ppose(0);
  //   p.y = ppose(1);
  //   p.z = ppose(2);
  //   trajROS.points.push_back(p);
  //   p.x = pose(0);
  //   p.y = pose(1);
  //   p.z = pose(2);
  //   trajROS.points.push_back(p);
  //   std_msgs::ColorRGBA color;
  //   color.r = r;
  //   color.g = g;
  //   color.b = b;
  //   color.a = 1;
  //   trajROS.colors.push_back(color);
  //   trajROS.colors.push_back(color);
  //   ppose = pose;
  //   pt = t;
  //   trajPub.publish(trajROS);
  // }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_visualization");
  ros::NodeHandle n("~");

  // n.param("mesh_resource", mesh_resource, std::string("package://odom_visualization/meshes/hummingbird.mesh"));
  n.param("mesh_resource", mesh_resource, std::string("package://odom_visualization/meshes/flymesh.dae"));

  n.param("origin", origin, false);
  n.param("robot_scale", scale, 2.0);
  n.param("frame_id", _frame_id, string("world"));
  n.param("cross_config", cross_config, false);

  ros::Subscriber sub_odom = n.subscribe("odom", 100, odom_callback);
  // posePub = n.advertise<geometry_msgs::PoseStamped>("pose", 100, true);
  // pathPub = n.advertise<nav_msgs::Path>("path", 100, true);
  // trajPub = n.advertise<visualization_msgs::Marker>("trajectory", 100, true);
  meshPub = n.advertise<visualization_msgs::Marker>("robot", 100, true);

  ros::spin();
  return 0;
}

/**
 * @brief leader estimator class
 *
 * leader state estimator
 *
 * @author Peixuan Shu <shupeixuan@qq.com>
 * 
 * First built on 2021.04.04
 */

#include "uavros_uavugv_sitl/leader_estimator.h"

using namespace Eigen;
using namespace std;

leaderEstimate::leaderEstimate(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh),nh_private_(nh_private)
{

  leader_state_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("leader_pose_estimate", 10);
  leaderPath_pub_ = nh_.advertise<nav_msgs::Path>("trajectory/leader_traj", 1); //to rviz 

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &leaderEstimate::cmdloop_cb, this);  // Define timer for constant loop rate  
  trajloop_timer_ = nh_.createTimer(ros::Duration(0.1), &leaderEstimate::trajloop_cb, this);  // Define timer for constant loop rate  


  nh_private_.param<double>("shape_omega", shape_omega_, 0.5);
  nh_private_.param<double>("shape_radius", shape_radius_, 1);
  nh_private_.param<double>("alt_sp", alt_sp, 2.5);

  leaderPos_ << 0.0, 0.0, 0.0;
  leaderVel_ << 0.0, 0.0, 0.0;
  leaderAcc_ << 0.0, 0.0, 0.0;
  dt_ = 0.0; time_now_ = 0.0; time_last_ = 0.0;

}

void leaderEstimate::cmdloop_cb(const ros::TimerEvent &event)
{
  time_now_ = ros::Time::now().toSec();
  dt_ = time_now_ - time_last_;
  time_last_ = time_now_;
  shapeCreator(time_now_);
  pubLeaderEstimation(leaderPos_, leaderVel_, leaderAcc_);
}

void leaderEstimate::trajloop_cb(const ros::TimerEvent &event)
{
  pubTrajectory();
}

void leaderEstimate::shapeCreator(double t)
{
  double theta;
  theta = shape_omega_ * t;
  leaderPos_(0) = shape_radius_ * cos(theta);
  leaderPos_(1) = shape_radius_ * sin(theta);
  leaderVel_(0) = - shape_radius_ * shape_omega_* sin(theta);
  leaderVel_(1) = shape_radius_ * shape_omega_* cos(theta);
  leaderAcc_(0) = - shape_radius_ * shape_omega_* shape_omega_ * cos(theta);
  leaderAcc_(1) = - shape_radius_ * shape_omega_* shape_omega_ * sin(theta);
}

void leaderEstimate::pubLeaderEstimation(const Eigen::Vector3d &state_pos, const Eigen::Vector3d &state_vel, const Eigen::Vector3d &state_acc)
{
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.position.x = state_pos(0); 
  msg.position.y = state_pos(1); 
  msg.position.z = alt_sp;  // only valid in xy plane
  msg.velocity.x = state_vel(0);
  msg.velocity.y = state_vel(1);
  msg.velocity.z = 0.0;
  msg.acceleration_or_force.x = state_acc(0); 
  msg.acceleration_or_force.y = state_acc(1); 
  msg.acceleration_or_force.z = 0.0; 
  leader_state_pub_.publish(msg);
}

void leaderEstimate::pubTrajectory()
{
  nav_msgs::Path refTrajectory_;
  int length = sizeof(trajPoseStamped) / sizeof(trajPoseStamped[0]);

  for (int i = 0; i < length-1; i ++)
  {
    trajPoseStamped[i] = trajPoseStamped[i+1];
    refTrajectory_.poses.push_back(trajPoseStamped[i]);
  }
  trajPoseStamped[length-1].header.stamp = ros::Time::now();
  trajPoseStamped[length-1].header.frame_id = "map";
  trajPoseStamped[length-1].pose.orientation.w = 1.0;
  trajPoseStamped[length-1].pose.orientation.x = 0.0;
  trajPoseStamped[length-1].pose.orientation.y = 0.0;
  trajPoseStamped[length-1].pose.orientation.z = 0.0;
  trajPoseStamped[length-1].pose.position.x = leaderPos_(0);
  trajPoseStamped[length-1].pose.position.y = leaderPos_(1);
  trajPoseStamped[length-1].pose.position.z = alt_sp;
  refTrajectory_.poses.push_back(trajPoseStamped[length-1]);
  refTrajectory_.header.stamp = ros::Time::now();
  refTrajectory_.header.frame_id = "map";

  leaderPath_pub_.publish(refTrajectory_);
}

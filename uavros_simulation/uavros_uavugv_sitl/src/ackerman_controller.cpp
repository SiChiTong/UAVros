/**
 * @brief ackerman controller class
 *
 * ackerman Controller to output setpoint
 *
 * @author Peixuan Shu <shupeixuan@qq.com>
 * 
 * First built on 2021.04.12
 */

#include "ackerman_controller.h"

using namespace Eigen;
using namespace std;

ackermanCtrl::ackermanCtrl (const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh),nh_private_(nh_private)
{
  ugvposeSub_ = nh_.subscribe("odom", 1, &ackermanCtrl::ugvpose_cb, this, ros::TransportHints().tcpNoDelay());
  leaderposeSub_ = nh_.subscribe("leader_pose_estimate", 1, &ackermanCtrl::leaderpose_cb, this, ros::TransportHints().tcpNoDelay()); 
  cmdSub_ = nh_.subscribe("/cmd", 1, &ackermanCtrl::cmd_cb, this, ros::TransportHints().tcpNoDelay()); 
  target_pose_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd_mux/output", 10);

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.05), &ackermanCtrl::cmdloop_cb, this);  // Define timer for constant loop rate, the max frequency beike car can control is 20Hz  

  nh_private_.param<double>("v_max", v_max_, 2.5);
  nh_private_.param<double>("steering_max_rad", delta_max_, 0.35);
  nh_private_.param<double>("Kp_x", Kpos_x_, 0.5);
  nh_private_.param<double>("Kp_y", Kpos_y_, 0.5);
  nh_private_.param<double>("Kp_heading", beta_, 1.0);
  nh_private_.param<double>("init_phi", heading_, 0);
  nh_private_.param<double>("wheelbase", L_, 0.335);

  Kpos_ << Kpos_x_, Kpos_y_, 0.0;
  ugvAtt_ << 0.0, 0.0, 0.0, 1.0;
  ugvPos_ << 0.0, 0.0, 0.0;
  ugvVel_ << 0.0, 0.0, 0.0;
  leaderPos_ << 0.0, 0.0, 0.0;
  leaderVel_ << 0.0, 0.0, 0.0;
  leaderAcc_ << 0.0, 0.0, 0.0; 
  command_ = 0;
  start_flag_ = 0;
  start_time_ = 0;

}

void ackermanCtrl::cmdloop_cb(const ros::TimerEvent &event)
{
  switch (command_)
  {
  case 0:
    v_sp_ = 0;
    delta_sp_ = 0;
    break;
  case 1:
    if (start_flag_ == 0)
    {
      start_time_ = ros::Time::now().toSec();
    }
    start_flag_ = 1;
    t_ = ros::Time::now().toSec() - start_time_;
    computeSetpoint(v_sp_, delta_sp_, leaderPos_, leaderVel_, leaderAcc_); 
    break; 
  default:
    command_ = 0; //any other command will leader to case 0
    cout << "ugv controller: unknown command, switch to command 0" << endl;
    break;
  }
  pubSetpoint(v_sp_, delta_sp_);
}

void ackermanCtrl::computeSetpoint(double &v_cmd, double &delta_cmd, const Eigen::Vector3d &target_pos,
                                       const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc)
{
  const Eigen::Vector3d pos_error = target_pos - ugvPos_;
  const double w_c = (target_vel(0)*target_acc(1)-target_vel(1)*target_acc(0))
                        /sqrt(target_vel(0)*target_vel(0)+target_vel(1)*target_vel(1));
  Eigen::Vector3d u;
  u = Kpos_.asDiagonal() * pos_error  + target_vel;
  v_cmd = u(0)*cos(heading_) + u(1)*sin(heading_);
  v_cmd = max(-v_max_, min(v_max_, v_cmd));
  delta_cmd = atan(L_*beta_*(-u(0)*sin(heading_)+u(1)*cos(heading_) + w_c)/v_cmd);
  delta_cmd = max(-delta_max_, min(delta_max_, delta_cmd));
}

void ackermanCtrl::pubSetpoint(const double &v_sp, const double &delta_sp)
{
  ackermann_msgs::AckermannDriveStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.drive.steering_angle = delta_sp;
  msg.drive.speed = v_sp;
  msg.drive.steering_angle_velocity = 0.0;
  msg.drive.acceleration = 0.0;
  msg.drive.jerk = 0.0;
  target_pose_pub_.publish(msg);
}

void ackermanCtrl::ugvpose_cb(const nav_msgs::Odometry &msg)
{
  ugvPos_(0) = msg.pose.pose.position.x;
  ugvPos_(1) = msg.pose.pose.position.y;
  ugvPos_(2) = msg.pose.pose.position.z;
  ugvAtt_(0) = msg.pose.pose.orientation.x;
  ugvAtt_(1) = msg.pose.pose.orientation.y;
  ugvAtt_(2) = msg.pose.pose.orientation.z;
  ugvAtt_(3) = msg.pose.pose.orientation.w;
  ugvVel_(0) = msg.twist.twist.linear.x;
  ugvVel_(1) = msg.twist.twist.linear.y;
  ugvVel_(2) = msg.twist.twist.linear.z;
  Eigen::Quaterniond quaternion(ugvAtt_);
  Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(2,1,0);
  if (abs(eulerAngle(1)) > 3) //pitch should be within (-pi/2,pi/2)
    heading_ = eulerAngle(0) - M_PI;
  else
    heading_ = eulerAngle(0);
   //cout << "heading: " << heading_*180/M_PI << endl;
}

void ackermanCtrl::leaderpose_cb(const mavros_msgs::PositionTarget &msg)
{
  leaderPos_(0) = msg.position.x;
  leaderPos_(1) = msg.position.y;
  leaderPos_(2) = msg.position.z;
  leaderVel_(0) = msg.velocity.x;
  leaderVel_(1) = msg.velocity.y;
  leaderVel_(2) = msg.velocity.z;
  leaderAcc_(0) = msg.acceleration_or_force.x;
  leaderAcc_(1) = msg.acceleration_or_force.y;
  leaderAcc_(2) = msg.acceleration_or_force.z;
}

void ackermanCtrl::cmd_cb(const std_msgs::Int32 &msg)
{
	command_ = msg.data;
	cout << "ugv controller receive command: " << command_ << endl;
}

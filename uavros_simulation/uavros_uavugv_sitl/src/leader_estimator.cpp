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
  nh_private_.param<double>("shape_omega", shape_omega_, 0.5);
  nh_private_.param<double>("shape_radius", shape_radius_, 1);
  nh_private_.param<double>("alt_sp", alt_sp, 2.5);
  nh_private_.param<double>("alpha", alpha_, 1); // coefficient of the distributed estimator
  nh_private_.param<int>("neighbor_num", ng_num_, 1); // number of neighbor, only support 1 now
  nh_private_.param<string>("neighbor1_name", ng_name1_, "none");
  nh_private_.param<string>("neighbor2_name", ng_name2_, "none");

  cout << "ng_num_: " << ng_num_ <<endl;
  cout << "ng_name1_: " << ng_name1_ <<endl;
  cout << "alpha: " << alpha_ <<endl;

  ng_estimateSub1_ = nh_.subscribe("/"+ng_name1_+"/leader_pose_estimate", 1, &leaderEstimate::ng_estimate_cb1, this, ros::TransportHints().tcpNoDelay()); 
  leader_state_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("leader_pose_estimate", 10);
  leaderPath_pub_ = nh_.advertise<nav_msgs::Path>("trajectory/leader_traj", 1); //to rviz 

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &leaderEstimate::cmdloop_cb, this);  // Define timer for constant loop rate  
  trajloop_timer_ = nh_.createTimer(ros::Duration(0.1), &leaderEstimate::trajloop_cb, this);  // Define timer for constant loop rate  


  leaderPos_ << 0.0, 0.0, 0.0;
  leaderVel_ << 0.0, 0.0, 0.0;
  leaderAcc_ << 0.0, 0.0, 0.0;
  dt_ = 0.0; time_now_ = 0.0; time_last_ = 0.0;

  A0_ << 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 0.0;
        
  KP_ << 1.732, 0.0,
        1, 0.0,
        0.0, 1.732,
        0.0, 1; // gain matrix of the distributed estimator
  // column vector by default      
  q_hat_ << 0.0, 0.0, 0.0, 0.0; //estimation of leader state, [px;vx;py;vy]
  //TODO: q_hat_ should be the current pos of the uav in the beginning!
  q_hat_last_ << 0.0, 0.0, 0.0, 0.0; //estimation of leader state in the last loop, pos and vel
  y_hat1_ << 0.0, 0.0; //neighbor_1's estimation of leader output, pos [px;py]
  y_hat2_ << 0.0, 0.0; //neighbor_2's estimation of leader output, pos
}

void leaderEstimate::cmdloop_cb(const ros::TimerEvent &event)
{
  time_now_ = ros::Time::now().toSec();
  dt_ = time_now_ - time_last_;
  time_last_ = time_now_;
  // shapeCreator(time_now_);  //publish a circle shaped leader pos
  distributed_estimator(dt_);
  pubLeaderEstimation(leaderPos_, leaderVel_, leaderAcc_);
}

void leaderEstimate::trajloop_cb(const ros::TimerEvent &event)
{
  pubTrajectory();
}

void leaderEstimate::distributed_estimator(double dt)
{
  Eigen::Vector4d dq_hat;
  Eigen::Vector2d y_hat;
  y_hat(0) = q_hat_last_(0); //estimate px of leader
  y_hat(1) = q_hat_last_(2); //estimate py of leader
  if (ng_num_ == 1)
    {
      dq_hat = A0_*q_hat_last_ - alpha_*KP_*(y_hat - y_hat1_); 
    }
  else 
    {
      // TODO: if neighbor_num ==0 or more than 1
    }
  q_hat_ = q_hat_last_ + dq_hat * dt; //renew q_hat_ 
  q_hat_last_ = q_hat_;
  leaderPos_(0) = q_hat_(0);
  leaderPos_(1) = q_hat_(2);
  leaderVel_(0) = q_hat_(1);
  leaderVel_(1) = q_hat_(3);
  leaderAcc_(0) = (A0_*q_hat_)(1);
  leaderAcc_(1) = (A0_*q_hat_)(3);  
}

void leaderEstimate::shapeCreator(double t)
{ //circle creator
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
{ //for rviz to display the path
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

void leaderEstimate::ng_estimate_cb1(const mavros_msgs::PositionTarget &msg)
{ // only get neighbor's estimation of leader's posXY by communication
  y_hat1_(0) = msg.position.x;
  y_hat1_(1) = msg.position.y;
}
/**
 * @brief leader estimator class
 *
 * leader state estimator
 *
 * @author Peixuan Shu <shupeixuan@qq.com>
 * 
 * First built on 2021.04.04
 */

#include "leader_estimator.h"

using namespace Eigen;
using namespace std;

leaderEstimate::leaderEstimate(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh),nh_private_(nh_private)
{
  nh_private_.param<double>("shape_omega", shape_omega_, 0.5);
  nh_private_.param<double>("shape_radius", shape_radius_, 1);
  nh_private_.param<double>("shape_x0", shape_x0_, 0);
  nh_private_.param<double>("shape_y0", shape_y0_, 0);
  nh_private_.param<double>("alt_sp", alt_sp, 2.5);
  nh_private_.param<double>("alpha", alpha_, 1); // coefficient of the distributed estimator
  nh_private_.param<double>("esm_x_init", esm_x_init_, 0.0);
  nh_private_.param<double>("esm_y_init", esm_y_init_, 0.0);
  nh_private_.param<double>("virtual_leader_vx", l_vx_, 0.2);
  nh_private_.param<double>("virtual_leader_vy", l_vy_, 0.2);
  nh_private_.param<int>("neighbor_num", ng_num_, 1); // number of neighbor, only support 1 now
  nh_private_.param<int>("rviz_path_length", trajpose_window_, 70); // length num of leaderPoseEstimate nav path in rviz
  nh_private_.param<string>("neighbor1_name", ng_name1_, "none"); //"virtual_leader**" means it knows the state of leader
  nh_private_.param<string>("neighbor2_name", ng_name2_, "none");

  cout << "ng_num_: " << ng_num_ <<endl;
  cout << "ng_name1_: " << ng_name1_ <<endl;
  cout << "alpha: " << alpha_ <<endl;

  if((ng_name1_ != "virtual_leader_line") && (ng_name1_ != "virtual_leader_circle"))
  {
    ng_estimateSub1_ = nh_.subscribe("/"+ng_name1_+"/leader_pose_estimate", 1, &leaderEstimate::ng_estimate_cb1, 
                                                                    this, ros::TransportHints().tcpNoDelay()); 
  }
  cmdSub_ = nh_.subscribe("/cmd", 1, &leaderEstimate::cmd_cb, this, ros::TransportHints().tcpNoDelay()); 
  leader_state_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("leader_pose_estimate", 10);
  leaderPath_pub_ = nh_.advertise<nav_msgs::Path>("trajectory/leader_traj", 1); //to rviz 

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.05), &leaderEstimate::cmdloop_cb, this);  // Define timer for constant loop rate, 100Hz is too high and unnecessary  
  trajloop_timer_ = nh_.createTimer(ros::Duration(0.1), &leaderEstimate::trajloop_cb, this);  // Define timer for constant loop rate  


  leaderPos_ << esm_x_init_, esm_y_init_, 0.0;
  leaderVel_ << 0.0, 0.0, 0.0;
  leaderAcc_ << 0.0, 0.0, 0.0;
  dt_ = 0.0; time_now_ = 0.0; time_last_ = 0.0; 
  t1_start_ = 0.0; t1_accum_ = 0.0; t_ = 0.0;
  command_ = 0; state_num_ = 0;

  A0_ << 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 0.0;
        
  KP_ << 1.732, 0.0,
        1, 0.0,
        0.0, 1.732,
        0.0, 1; // gain matrix of the distributed estimator
  // column vector by default      
  q_hat_ << esm_x_init_, 0.0, esm_y_init_, 0.0; //estimation of leader state, [px;vx;py;vy]
  //TODO: q_hat_ should be the current pos of the uav in the beginning!
  q_hat_last_ << esm_x_init_, 0.0, esm_y_init_, 0.0; //estimation of leader state in the last loop, pos and vel
  y_hat1_ << 0.0, 0.0; //neighbor_1's estimation of leader output, pos [px;py]
  y_hat2_ << 0.0, 0.0; //neighbor_2's estimation of leader output, pos
}

void leaderEstimate::cmdloop_cb(const ros::TimerEvent &event)
{
  time_now_ = ros::Time::now().toSec();
  dt_ = time_now_ - time_last_;
  time_last_ = time_now_;

  switch (command_)
  {
  case 0: //waiting and suspending
    if (state_num_ == 1)
    {//if change mode from 1, latch the t_ to t1_accum_(accumulate)
      t1_accum_ = t_;
    }
    state_num_ = 0;
    break;

  case 1:
    if (state_num_ == 0)
    {//record the time when case 1 start from case 0
      t1_start_ = ros::Time::now().toSec();
    }
    state_num_ = 1;
    t_ = ros::Time::now().toSec() - t1_start_ + t1_accum_;
    //t_ is the total time in case 1 despite several case switches
    if (ng_name1_ == "virtual_leader_circle")
    {//generate circle leader pose directly,not from estimator
      circleCreator(t_);
    }
    else
    {
      if (ng_name1_ == "virtual_leader_line")
      {
        y_hat1_(0) = l_vx_ * t_ + shape_x0_;
        y_hat1_(1) = l_vy_ * t_ + shape_y0_;
      }
      distributed_estimator(dt_);
    }
    break; 

  default:
    command_ = 0; //any other command will leader to case 0
    cout << "leader estimator: unknown command, switch to command 0" << endl;
    break;
  }

  pubLeaderEstimation(leaderPos_, leaderVel_, leaderAcc_);
}

void leaderEstimate::trajloop_cb(const ros::TimerEvent &event)
{
  pubTrajectory();// to rviz
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
      // TODO: if neighbor_num == 0 or more than 1
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

void leaderEstimate::circleCreator(double t)
{ //circle creator
  double theta;
  theta = shape_omega_ * t;
  leaderPos_(0) = shape_radius_ * cos(theta) + shape_x0_;
  leaderPos_(1) = shape_radius_ * sin(theta) + shape_y0_;
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
  nav_msgs::Path path_msg_;
  geometry_msgs::PoseStamped TrajPose_;
  TrajPose_.header.stamp = ros::Time::now();
  TrajPose_.header.frame_id = "map";
  TrajPose_.pose.orientation.w = 1.0;
  TrajPose_.pose.orientation.x = 0.0;
  TrajPose_.pose.orientation.y = 0.0;
  TrajPose_.pose.orientation.z = 0.0;
  TrajPose_.pose.position.x = leaderPos_(0);
  TrajPose_.pose.position.y = leaderPos_(1);
  TrajPose_.pose.position.z = alt_sp;  

  TrajPoseHistory_vector_.insert(TrajPoseHistory_vector_.begin(), TrajPose_);
  if (TrajPoseHistory_vector_.size() > trajpose_window_)
  {
    TrajPoseHistory_vector_.pop_back();
  }

  path_msg_.header.stamp = ros::Time::now();
  path_msg_.header.frame_id = "map";
  path_msg_.poses = TrajPoseHistory_vector_;

  leaderPath_pub_.publish(path_msg_);
}

void leaderEstimate::ng_estimate_cb1(const mavros_msgs::PositionTarget &msg)
{ // only get neighbor's estimation of leader's posXY by communication
  y_hat1_(0) = msg.position.x;
  y_hat1_(1) = msg.position.y;
}

void leaderEstimate::cmd_cb(const std_msgs::Int32 &msg)
{
	command_ = msg.data;
	cout << "leader estimator receive command: " << command_ << endl;
}

/**
 * @brief UAV controller class
 *
 * UAV controller to output acceleration setpoint
 *
 * @author Peixuan Shu <shupeixuan@qq.com>
 * 
 * First built on 2021.04.02
 */

#include "uavros_uavugv_sitl/uav_controller.h"

using namespace Eigen;
using namespace std;

uavCtrl::uavCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh),nh_private_(nh_private)
{
  mavposeSub_ = nh_.subscribe("mavros/local_position/pose", 1, &uavCtrl::mavpose_cb, this, ros::TransportHints().tcpNoDelay());
  mavtwistSub_ = nh_.subscribe("mavros/local_position/velocity_local", 1, &uavCtrl::mavtwist_cb, this, ros::TransportHints().tcpNoDelay());
  leaderposeSub_ = nh_.subscribe("leader_pose_estimate", 1, &uavCtrl::leaderpose_cb, this, ros::TransportHints().tcpNoDelay()); 
  cmdSub_ = nh_.subscribe("/cmd", 1, &uavCtrl::cmd_cb, this, ros::TransportHints().tcpNoDelay()); 
  px4stateSub_ = nh_.subscribe("mavros/state", 1, &uavCtrl::px4state_cb, this, ros::TransportHints().tcpNoDelay());
  target_pose_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  setMode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &uavCtrl::cmdloop_cb, this);  // Define timer for constant loop rate  

  nh_private_.param<double>("alt_sp", alt_sp, 2.5);
  nh_private_.param<double>("Kp_x", Kpos_x_, 0.5);
  nh_private_.param<double>("Kp_y", Kpos_y_, 0.5);
  nh_private_.param<double>("Kp_z", Kpos_z_, 0.0);
  nh_private_.param<double>("Kv_x", Kvel_x_, 0.2);
  nh_private_.param<double>("Kv_y", Kvel_y_, 0.2);
  nh_private_.param<double>("Kv_z", Kvel_z_, 0.0);
  nh_private_.param<double>("init_x", init_x_, 0);
  nh_private_.param<double>("init_y", init_y_, 0);
  nh_private_.param<double>("h_omega", h_omega_, 0);
  nh_private_.param<double>("h_radius", h_radius_, 0);

  Kpos_ << Kpos_x_, Kpos_y_, Kpos_z_;
  Kvel_ << Kvel_x_, Kvel_y_, Kvel_z_;
  mavAtt_ << 1.0, 0.0, 0.0, 0.0;
  mavPos_ << 0.0, 0.0, 0.0;
  mavVel_ << 0.0, 0.0, 0.0;
  leaderPos_ << 0.0, 0.0, 0.0;
  leaderVel_ << 0.0, 0.0, 0.0;
  leaderAcc_ << 0.0, 0.0, 0.0; 
  acc_sp << 0.0, 0.0, 0.0;
  hPos_ << 0.0, 0.0, 0.0;
  hVel_ << 0.0, 0.0, 0.0;
  hAcc_ << 0.0, 0.0, 0.0;
  command_ = 0;
  start_flag_ = 0;
  start_time_ = 0;

}

void uavCtrl::cmdloop_cb(const ros::TimerEvent &event)
{
  switch (command_)
  {
  case 0:
    break;
  case 1:
    if (start_flag_ == 0)
    {
      start_time_ = ros::Time::now().toSec();
    }
    start_flag_ = 1;
    if (px4_state_.mode != "OFFBOARD")
    {
        mode_cmd_.request.custom_mode = "OFFBOARD";
				setMode_client_.call(mode_cmd_);
    }
    t_ = ros::Time::now().toSec() - start_time_;
    double theta;
    theta = h_omega_ * t_;
    hPos_(0) = h_radius_ * cos(theta);
    hPos_(1) = h_radius_ * sin(theta);
    hVel_(0) = - h_radius_ * h_omega_* sin(theta);
    hVel_(1) = h_radius_ * h_omega_* cos(theta);
    hAcc_(0) = - h_radius_ * h_omega_* h_omega_ * cos(theta);
    hAcc_(1) = - h_radius_ * h_omega_* h_omega_ * sin(theta);

    computeAccCmd(acc_sp, leaderPos_ + hPos_, leaderVel_ + hVel_, leaderAcc_ + hAcc_); 
    //compute acceleration setpoint command by PD controller algorithm;  
    break; 
  default:
    cout << "error command!" << endl;
    break;
  }

  pubAccCmd(acc_sp);
}

void uavCtrl::computeAccCmd(Eigen::Vector3d &acc_cmd, const Eigen::Vector3d &target_pos,
                                       const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc)
{
  const Eigen::Vector3d pos_error = target_pos - mavPos_;
  const Eigen::Vector3d vel_error = target_vel - mavVel_;

  acc_cmd = Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error + target_acc;
}

void uavCtrl::pubAccCmd(const Eigen::Vector3d &cmd_acc)
{
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.coordinate_frame = 1; //pub in ENU local frame;
  msg.type_mask = 0b110000111011; // pub az+ay+ax+pz, az is feedforward acc. ignore yaw and yaw rate
  msg.position.z = alt_sp; // pub local z altitude setpoint
  msg.acceleration_or_force.x = cmd_acc(0); //pub ax
  msg.acceleration_or_force.y = cmd_acc(1); //pub ay
  msg.acceleration_or_force.z = 0; //az feedforward = 0
  target_pose_pub_.publish(msg);
}

void uavCtrl::mavpose_cb(const geometry_msgs::PoseStamped &msg)
{
  mavPos_(0) = msg.pose.position.x;
  mavPos_(1) = msg.pose.position.y;
  mavPos_(2) = msg.pose.position.z;
  mavAtt_(0) = msg.pose.orientation.w;
  mavAtt_(1) = msg.pose.orientation.x;
  mavAtt_(2) = msg.pose.orientation.y;
  mavAtt_(3) = msg.pose.orientation.z;
}

void uavCtrl::mavtwist_cb(const geometry_msgs::TwistStamped &msg)
{
  mavVel_(0) = msg.twist.linear.x;
  mavVel_(1) = msg.twist.linear.y;
  mavVel_(2) = msg.twist.linear.z;
}

void uavCtrl::leaderpose_cb(const mavros_msgs::PositionTarget &msg)
{
  leaderPos_(0) = msg.position.x - init_x_; //in gazebo, the UAV origin is at the takeoff home postision
  leaderPos_(1) = msg.position.y - init_y_;
  leaderPos_(2) = msg.position.z;
  leaderVel_(0) = msg.velocity.x;
  leaderVel_(1) = msg.velocity.y;
  leaderVel_(2) = msg.velocity.z;
  leaderAcc_(0) = msg.acceleration_or_force.x;
  leaderAcc_(1) = msg.acceleration_or_force.y;
  leaderAcc_(2) = msg.acceleration_or_force.z;
}

void uavCtrl::cmd_cb(const std_msgs::Int32 &msg)
{
	command_ = msg.data;
	cout << "receive command: " << command_ << endl;
}

void uavCtrl::px4state_cb(const mavros_msgs::State &msg)
{
	px4_state_ = msg;
}
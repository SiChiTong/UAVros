/**
 * @brief UAV controller class
 *
 * Flight controller to output velocity setpoint
 *
 * @author Peixuan Shu <shupeixuan@qq.com>
 * 
 * First built on 2021.07.10
 */

#include "uav_controller.h"

using namespace Eigen;
using namespace std;

uavCtrl::uavCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh),nh_private_(nh_private)
{
  mavposeSub_ = nh_.subscribe("mavros/local_position/pose", 1, &uavCtrl::mavpose_cb, this, ros::TransportHints().tcpNoDelay());
  mavtwistSub_ = nh_.subscribe("mavros/local_position/velocity_local", 1, &uavCtrl::mavtwist_cb, this, ros::TransportHints().tcpNoDelay());
  //leaderposeSub_ = nh_.subscribe("leader_pose_estimate", 1, &uavCtrl::leaderpose_cb, this, ros::TransportHints().tcpNoDelay()); 
  //cmdSub_ = nh_.subscribe("/cmd", 1, &uavCtrl::cmd_cb, this, ros::TransportHints().tcpNoDelay()); 
  px4stateSub_ = nh_.subscribe("mavros/state", 1, &uavCtrl::px4state_cb, this, ros::TransportHints().tcpNoDelay());
  jc_cmdSub_ = nh_.subscribe("/jc_cmd", 1, &uavCtrl::jc_cmd_cb, this, ros::TransportHints().tcpNoDelay());
  target_pose_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
  en_track_pub_ = nh_.advertise<std_msgs::String>("gimbal/gimbal_en", 10);

  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  setMode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  //precise_landing_client_ = nh_.serviceClient<std_srvs::SetBool>("precise_landing");

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.05), &uavCtrl::cmdloop_cb, this);  // Define timer for constant loop rate  

  nh_private_.param<double>("arrive_alt", arrive_alt_, 7.0);
  nh_private_.param<double>("track_alt", track_alt_, 5.0);
  nh_private_.param<double>("hover_yaw_rad", yaw_sp_, 0.0);  
  nh_private_.param<double>("Kp", Kp_, 1.0);
  nh_private_.param<double>("Kd", Kd_, 0.0);
  nh_private_.param<double>("Ki", Ki_, 0.0);
  nh_private_.param<double>("vxy_max", vxy_max_, 5.0);
  nh_private_.param<double>("car_initposx", car_initposx_, 1.0);
  nh_private_.param<double>("car_initposy", car_initposy_, 1.0);
  
  cout << "arrive_alt: " << arrive_alt_ << endl;
  cout << "track_alt: " << track_alt_ << endl;
  cout << "Kp: " << Kp_ << endl;
  cout << "Kd: " << Kd_ << endl;
  cout << "Ki: " << Ki_ << endl;
  cout << "vxy_max: " << vxy_max_ << endl;
  takeoff_triggered_ = false;
  offboard_triggered_ = false;
  return_triggered_ = false;

  controller_state = PREPARE;
  last_state = PREPARE;
  command_ = BLANK;
}

void uavCtrl::cmdloop_cb(const ros::TimerEvent &event)
{
  switch (controller_state)
  {
  case PREPARE: //waiting and initilizing
    //VxyPz_sp << 0.0, 0.0, alt_sp;
    //pubVxyPzCmd(VxyPz_sp);
    string_msg_.data = "CLOSE"; //close camera tracking
    en_track_pub_.publish(string_msg_);
    if (command_ == LAUNCH)
    {
      controller_state = TAKEOFF;
    }
    else if(command_ != BLANK)
    {
      cout << "warning: the jc2fk.txt content is not wait!";
    }
    last_state = PREPARE;
    cout << "PREPARE" << endl;
    break;
  
  case TAKEOFF:
    if(takeoff_triggered_ == false)
    {
      mode_cmd_.request.custom_mode = "AUTO.TAKEOFF";
		  if(setMode_client_.call(mode_cmd_))
      {
        takeoff_triggered_ = true;
        cout << "takeoff mode triggered, please arm" << endl;
      }
      else
      {
        cout << "takeoff trigger fail,arm first" << endl;
      }
    }
    else
    {
      if ((px4_state_.mode != "AUTO.TAKEOFF")&&(mavPos_(2) > 2.0))
      {controller_state = FLYTOCAR;}
    }    
    last_state = TAKEOFF;
    cout << "TAKEOFF" << endl;
    break;

  case FLYTOCAR:
    PxyPz_sp << car_initposx_, car_initposy_, arrive_alt_;
    pubPxyPzCmd(PxyPz_sp);
    if((px4_state_.mode != "OFFBOARD")&&(offboard_triggered_ == false))
    {
      mode_cmd_.request.custom_mode = "OFFBOARD";
			if(setMode_client_.call(mode_cmd_))
      {
        offboard_triggered_ = true;
        cout << "offboard mode triggered" << endl;
      }      
    }
    if( (fabs(mavPos_(0)-car_initposx_) < 0.2) && (fabs(mavPos_(1)-car_initposy_) < 0.2) )
    {
      controller_state = HOVER_ON_CAR;
    }
    last_state = FLYTOCAR;
    cout << "FLYTOCAR" << endl;
    break;

  case HOVER_ON_CAR:
    string_msg_.data = "OPEN";
    en_track_pub_.publish(string_msg_); //open camera tracking
    PxyPz_sp << car_initposx_, car_initposy_, track_alt_;
    //yaw_sp_ = 0.0; //TODO: calculate vertical to velocity or car heading
    pubPxyzYawCmd(PxyPz_sp,yaw_sp_);
    if((fabs(mavPos_(2)-track_alt_) < 0.3))
    {
      controller_state = TRACK;
    }
    last_state = HOVER_ON_CAR;
    cout << "HOVER_ON_CAR" << endl;
    break;

  case TRACK:
    if (command_ == RETURN)
    {
      controller_state = FINISH;
    }
    last_state = TRACK;
    cout << "TRACK" << endl;
    break;

  case FINISH:
    if((px4_state_.mode != "AUTO.RTL")&&(return_triggered_ == false))
    {
      mode_cmd_.request.custom_mode = "AUTO.RTL";
			if(setMode_client_.call(mode_cmd_))
      {
        return_triggered_ = true;
        cout << "return mode triggered" << endl;
      }      
    }
    last_state = FINISH;
    cout << "RETURN" << endl;
    break;

/*
  case CONTROL_FLY:
    if (last_state == LAND)
    {
      controller_state = LAND;
      last_state = LAND;
      cout << "reject control_flying from LAND, keep LAND" << endl;
      break;
    }
    if (last_state == HOVER)
    {
      t1_start_ = ros::Time::now().toSec();
    }

    t_ = ros::Time::now().toSec() - t1_start_ + t1_accum_;
    double theta;
    theta = h_omega_ * t_;
    hPos_(0) = h_radius_ * cos(theta + h_phi_);
    hPos_(1) = h_radius_ * sin(theta + h_phi_);
    hVel_(0) = - h_radius_ * h_omega_* sin(theta + h_phi_);
    hVel_(1) = h_radius_ * h_omega_* cos(theta + h_phi_);
    hAcc_(0) = - h_radius_ * h_omega_* h_omega_ * cos(theta + h_phi_);
    hAcc_(1) = - h_radius_ * h_omega_* h_omega_ * sin(theta + h_phi_);

    computeAccCmd(acc_sp, leaderPos_ + hPos_, leaderVel_ + hVel_, leaderAcc_ + hAcc_); 
    //compute acceleration setpoint command by PD controller algorithm;  
    pubAccCmd(acc_sp);
    last_state = CONTROL_FLY;
    break; 

  case LAND:
    //do not send setpoint command
    if(last_state == CONTROL_FLY)
    {
      controller_state = HOVER;
      last_state = CONTROL_FLY;
      cout << "reject landing from CONTROL_FLY, switch to HOVER" << endl;
    }
    else if(last_state == HOVER)
    {
      std_srvs::SetBool land_service_cmd;
      land_service_cmd.request.data = true;
      if(precise_landing_client_.call(land_service_cmd))
      {
        last_state = LAND;
        cout << "land service call is success," << endl;
      }
      else
      {
        controller_state = HOVER;
        last_state = HOVER; 
        cout << "land service call failed, switch to HOVER" << endl;
      }
    }
    else
    {
      last_state = LAND; 
      //if last state is land, do not call land service repeatedly
    }
    break;
*/
  default:
    cout << "uav controller: error controller state!" << endl;
    break;
  }
}

/*
void uavCtrl::computeAccCmd(Eigen::Vector3d &acc_cmd, const Eigen::Vector3d &target_pos,
                                       const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc)
{
  const Eigen::Vector3d pos_error = target_pos - mavPos_;
  const Eigen::Vector3d vel_error = target_vel - mavVel_;

  acc_cmd = Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error + target_acc;
  acc_cmd(0) = max(-axy_max_, min(axy_max_, acc_cmd(0)));
  acc_cmd(1) = max(-axy_max_, min(axy_max_, acc_cmd(1)));
}
*/
void uavCtrl::pubPxyPzCmd(const Eigen::Vector3d &cmd_p)
{
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.coordinate_frame = 1; //pub in ENU local frame;
  msg.type_mask = 0b110111111000; // pub pz+py+px. ignore yaw and yaw rate
  msg.position.x = cmd_p(0); //pub px
  msg.position.y = cmd_p(1); //pub py
  msg.position.z = cmd_p(2); // pub local z altitude setpoint
  target_pose_pub_.publish(msg);
}

void uavCtrl::pubPxyzYawCmd(const Eigen::Vector3d &cmd_p, const double &yaw_sp)
{
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.coordinate_frame = 1; //pub in ENU local frame;
  msg.type_mask = 0b100111111000; // pub pz+py+px. ignore yaw and yaw rate
  msg.position.x = cmd_p(0); //pub px
  msg.position.y = cmd_p(1); //pub py
  msg.position.z = cmd_p(2); // pub local z altitude setpoint
  msg.yaw = yaw_sp;
  target_pose_pub_.publish(msg);
}

void uavCtrl::pubVxyPzCmd(const Eigen::Vector3d &cmd_sp)
{
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.coordinate_frame = 1; //pub in ENU local frame;
  msg.type_mask = 0b110111000011; // pub vx+vy+vz+pz, vz is feedforward vel. Ignore yaw and yaw rate
  msg.velocity.x = cmd_sp(0); //pub vx
  msg.velocity.y = cmd_sp(1); //pub vy
  msg.velocity.z = 0; //pub vz
  msg.position.z = cmd_sp(2); // pub local z altitude setpoint
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

/*
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
*/

void uavCtrl::jc_cmd_cb(const std_msgs::Int32 &msg)
{
  int jc_cmd;
  jc_cmd = msg.data;
	//cout << "uav controller receive command: " << jc_cmd << endl;
  if(jc_cmd == 0) {}
  else if(jc_cmd == 1) {command_ = LAUNCH;}
  else if(jc_cmd == 2) {command_ = FLYUP;}
  else if(jc_cmd == 3) {command_ = TO_ONE;}
  else if(jc_cmd == 4) {command_ = TO_TWO;}
  else if(jc_cmd == 5) {command_ = TO_THREE;}
  else if(jc_cmd == 6) {command_ = RETURN;}
  else {command_ = BLANK; cout << "uav controller: unknown jc_cmd" << endl;}
}

void uavCtrl::px4state_cb(const mavros_msgs::State &msg)
{
	px4_state_ = msg;
}

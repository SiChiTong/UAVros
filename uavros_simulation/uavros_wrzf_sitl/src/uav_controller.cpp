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
  gimbalSub_ = nh_.subscribe("/gimbal/gimbal_state", 1, &uavCtrl::gimbal_cb, this, ros::TransportHints().tcpNoDelay());
  jc_cmdSub_ = nh_.subscribe("/jc_cmd", 1, &uavCtrl::jc_cmd_cb, this, ros::TransportHints().tcpNoDelay());
  target_pose_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
  en_track_pub_ = nh_.advertise<std_msgs::String>("gimbal/gimbal_cmd", 10);

  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  setMode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  //precise_landing_client_ = nh_.serviceClient<std_srvs::SetBool>("precise_landing");

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &uavCtrl::cmdloop_cb, this);  // Define timer for constant loop rate  

  nh_private_.param<double>("arrive_alt", arrive_alt_, 7.0);
  nh_private_.param<double>("track_alt", track_alt_, 5.0);
  nh_private_.param<double>("hover_yaw_rad", hover_yaw_, 0.0);  
  nh_private_.param<double>("vxy_max", vxy_max_, 4.0);
  nh_private_.param<double>("car_initposx", car_initposx_, 1.0);
  nh_private_.param<double>("car_initposy", car_initposy_, 1.0);
  
  cout << "arrive_alt: " << arrive_alt_ << endl;
  cout << "track_alt: " << track_alt_ << endl;
  cout << "Kp: " << Kp_ << endl;
  cout << "Kd: " << Kd_ << endl;
  cout << "Ki: " << Ki_ << endl;
  cout << "vxy_max: " << vxy_max_ << endl;

  Kp_ = 0.0;
  Ki_ = 0.0;
  Kd_ = 0.0;
  takeoff_triggered_ = false;
  offboard_triggered_ = false;
  return_triggered_ = false;

  pidx.error_last = 0;
  pidx.integral = 0;
  pidy.error_last = 0;
  pidy.integral = 0;

  error_pE_ = 0.1;
  error_pN_ = 0.1;
  VxyPz_sp_ << 0.0,0.0,track_alt_;

  yaw_sp_ = hover_yaw_;

  gim_yaw_ = 0.0;
  gim_pitch_ = 0.0;
  gim_servo_state_ = "init";
  gim_track_state_ = "init";

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
    if(gim_servo_state_ == "TRACK")
    {
      string_msg_.data = "CLOSE"; //close camera tracking
      en_track_pub_.publish(string_msg_);
    }
    if (command_ == LAUNCH)
    {
      controller_state = TAKEOFF;
      //controller_state = TRACK;
    }
    else if(command_ != BLANK)
    {
      cout << "warning: the jc2fk.txt content is not correct!" << endl;
    }
    last_state = PREPARE;
    cout << "PREPARE" << endl;
    break;
  
  case TAKEOFF:
  //TODO: under 2m, continuing to trigger takeoff
  /*
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
  */
    if ((px4_state_.mode != "AUTO.TAKEOFF")&&(mavPos_(2) < 2.0))
    {
      mode_cmd_.request.custom_mode = "AUTO.TAKEOFF";
      setMode_client_.call(mode_cmd_);
    }
    if ((px4_state_.mode != "AUTO.TAKEOFF")&&(mavPos_(2) > 2.0))
    {
      controller_state = FLYTOCAR;
      cout << "FLYTOCAR" << endl;
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
      cout << "HOVER_ON_CAR" << endl;
    }
    last_state = FLYTOCAR;
    //cout << "FLYTOCAR" << endl;
    break;

  case HOVER_ON_CAR:
    PxyPz_sp << car_initposx_, car_initposy_, track_alt_;
    //hover_yaw_ = 0.0; //TODO: calculate vertical to velocity or car heading
    pubPxyzYawCmd(PxyPz_sp,hover_yaw_);
    if((fabs(mavPos_(2)-track_alt_) < 0.3))
    {
      controller_state = TRACK;
      cout << "TRACK" << endl;
      //string_msg_.data = "OPEN";
      //en_track_pub_.publish(string_msg_); //open camera tracking
    }
    last_state = HOVER_ON_CAR;
    //cout << "HOVER_ON_CAR" << endl;
    break;

  case TRACK:
    if(px4_state_.mode != "OFFBOARD")
    {
      pidx.integral = 0.0;
      pidy.integral = 0.0;
      VxyPz_sp_ << 0.0,0.0,track_alt_;
      string_msg_.data = "CLOSE";
      en_track_pub_.publish(string_msg_); //close camera tracking
      cout << "CLOSE CAMERA TRACK"<< endl;
    }
    else //offboard
    {
      if((gim_servo_state_ != "TRACK")&&(gim_track_state_ == "HaveTarget"))
      {
        string_msg_.data = "OPEN";
        en_track_pub_.publish(string_msg_); //open camera tracking
        cout << "OPEN CAMERA TRACK"<< endl;
      }
      if((gim_servo_state_ == "TRACK")&&(gim_track_state_ != "Missing"))
      {
        computeError(gim_yaw_, gim_pitch_, mavAtt_, mavPos_); //compute error_pE_,error_pN_
        cout << "error_pE_: " << error_pE_ << ", error_pN_: " << error_pN_ << endl;
        computeVelCmd(VxyPz_sp_, error_pE_, error_pN_); //compute VxyPz_sp_
      }
    }
    yaw_sp_ = hover_yaw_;//TODO: calculate vertical to velocity or car heading
    //pubVxyPzYawCmd(VxyPz_sp_, yaw_sp_);
    pubVxyPzCmd(VxyPz_sp_);

    //TODO: if lost the target, fly following other UAVs
 
    if (command_ == RETURN)
    {
      controller_state = FINISH;
      cout << "RETURN" << endl;
    }
    //TODO: if exceed 6min, atuo return
    last_state = TRACK;
    //cout << "TRACK" << endl;
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

  default:
    cout << "uav controller: error controller state!" << endl;
    break;
  }
}


void uavCtrl::computeVelCmd(Eigen::Vector3d &vxypz_sp, const double &error_px, const double &error_py)
{
  pidx.error = error_px;
  pidx.derivative = error_px - pidx.error_last;
  pidx.integral += error_px;
  pidx.integral = max(-vxy_max_*0.8, min(pidx.integral, vxy_max_*0.8));//saturation resist
  pidx.error_last = error_px;
  vxypz_sp(0) = Kp_*pidx.error + Kd_*pidx.derivative + Ki_*pidx.integral;
  vxypz_sp(0) = max(-vxy_max_, min(vxy_max_, vxypz_sp(0)));

  pidy.error = error_py;
  pidy.derivative = error_py - pidy.error_last;
  pidy.integral += error_py;
  pidy.integral = max(-vxy_max_*0.8, min(pidy.integral, vxy_max_*0.8));//saturation resist
  pidy.error_last = error_py;
  vxypz_sp(1) = Kp_*pidy.error + Kd_*pidy.derivative + Ki_*pidy.integral;
  vxypz_sp(1) = max(-vxy_max_, min(vxy_max_, vxypz_sp(1)));

  vxypz_sp(2) = track_alt_;
}

void uavCtrl::computeError(const float &yaw, const float &pitch,
              const Eigen::Vector4d &uavatt, const Eigen::Vector3d &uavpos)
{
  Eigen::Vector3d campose_FLU, campose_ENU;
  campose_FLU << -cos(yaw)*sin(pitch), -sin(yaw), -cos(yaw)*cos(pitch); //in FLU body frame
  //yaw y, pitch z. out z, in y. y back, z left
  float qw = uavatt(0);
  float qx = uavatt(1);
  float qy = uavatt(2);
  float qz = uavatt(3);
  float height = uavpos(2);
  Eigen::Quaterniond q(qw,qx,qy,qz);
  Eigen::Matrix3d Rib = q.toRotationMatrix();
  campose_ENU = Rib*campose_FLU; //in ENU frame
  error_pE_ = fabs(height)/campose_ENU(2)*campose_ENU(0);
  error_pN_ = fabs(height)/campose_ENU(2)*campose_ENU(1);
}

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

void uavCtrl::pubVxyPzYawCmd(const Eigen::Vector3d &cmd_sp, const double &yaw_sp)
{
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.coordinate_frame = 1; //pub in ENU local frame;
  msg.type_mask = 0b100111000011; // pub vx+vy+vz+pz+yaw, vz is feedforward vel.
  msg.velocity.x = cmd_sp(0); //pub vx
  msg.velocity.y = cmd_sp(1); //pub vy
  msg.velocity.z = 0; //pub vz
  msg.position.z = cmd_sp(2); // pub local z altitude setpoint
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
  if(jc_cmd == 0) {command_ = BLANK;}
  else if(jc_cmd == 1) {command_ = LAUNCH;}
  else if(jc_cmd == 2) {command_ = FLYUP;}
  else if(jc_cmd == 3) {command_ = TO_ONE;}
  else if(jc_cmd == 4) {command_ = TO_TWO;}
  else if(jc_cmd == 5) {command_ = TO_THREE;}
  else if(jc_cmd == 6) {command_ = RETURN;}
  else {command_ = BLANK; cout << "uav controller: unknown jc_cmd" << endl;}
}

void uavCtrl::gimbal_cb(const uavros_msgs::TrackState &msg)
{
  gim_servo_state_ = msg.servo_state;
  gim_track_state_ = msg.track_state;
  gim_yaw_ = msg.yaw;
  gim_pitch_ = msg.pitch;
}

void uavCtrl::px4state_cb(const mavros_msgs::State &msg)
{
	px4_state_ = msg;
}

/*
void uavCtrl::dynamic_callback(const uavros_wrzf_sitl::dynamicConfig &config)
{
  Kp_ = config.P;
  Ki_ = config.I;
  Kd_ = config.D;
}
*/
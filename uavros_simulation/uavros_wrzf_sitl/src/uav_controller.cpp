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
  px4stateSub_ = nh_.subscribe("mavros/state", 1, &uavCtrl::px4state_cb, this, ros::TransportHints().tcpNoDelay());
  //gimbalSub_ = nh_.subscribe("/gimbal/gimbal_state", 1, &uavCtrl::gimbal_cb, this, ros::TransportHints().tcpNoDelay());
  jc_cmdSub_ = nh_.subscribe("/jc_cmd", 1, &uavCtrl::jc_cmd_cb, this, ros::TransportHints().tcpNoDelay());
  target_pose_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
  shape_yaw_pub_ = nh_.advertise<std_msgs::Float32>("/yaw_east_to_head_deg", 10);
  //en_track_pub_ = nh_.advertise<std_msgs::String>("gimbal/gimbal_cmd", 10);
	globalposSub_ = nh_.subscribe("mavros/global_position/global",10,&uavCtrl::globalpos_cb, this, ros::TransportHints().tcpNoDelay());
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  setMode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  target_err_Sub_ = nh_.subscribe("target_angle",10,&uavCtrl::targetCallback,this,ros::TransportHints().tcpNoDelay());
  //precise_landing_client_ = nh_.serviceClient<std_srvs::SetBool>("precise_landing");
  publoop_timer_ = nh_.createTimer(ros::Duration(0.05), &uavCtrl::publoop_cb, this);  // Define timer for pub loop rate  

  nh_private_.param<double>("loop_sec", loop_sec_, 0.05);
  cmdloop_timer_ = nh_.createTimer(ros::Duration(loop_sec_), &uavCtrl::cmdloop_cb, this);  // Define timer for constant loop rate  
  nh_private_.param<double>("arrive_alt", arrive_alt_, 10.0);
  nh_private_.param<double>("track_alt", track_alt_, 10.0);
  nh_private_.param<double>("hover_yaw_rad", hover_yaw_, 4.0);  
  nh_private_.param<double>("vxy_max", vxy_max_, 3.0);
  nh_private_.param<double>("acc_max",acc_max_, 6);
  nh_private_.param<double>("Kp", Kp_, 1.0);
  nh_private_.param<double>("Ki", Ki_, 0.0);
  nh_private_.param<double>("Kd", Kd_, 0.0);
  //nh_private_.param<double>("car_initposx", car_initposx_, 1.0);
  //nh_private_.param<double>("car_initposy", car_initposy_, 1.0);
  nh_private_.param<float>("car_initLat_deg", car_initLat_, 0.0);
  nh_private_.param<float>("car_initLon_deg", car_initLon_, 0.0);
  nh_private_.param<float>("car_Lat1_", car_Lat1_, 0.0);
  nh_private_.param<float>("car_Lon1_", car_Lon1_, 0.0);
  nh_private_.param<float>("car_Lat2_", car_Lat2_, 0.0);
  nh_private_.param<float>("car_Lon2_", car_Lon2_, 0.0);
  nh_private_.param<float>("car_Lat3_", car_Lat3_, 0.0);
  nh_private_.param<float>("car_Lon3_", car_Lon3_, 0.0);
  nh_private_.param<float>("car_Lat4_", car_Lat4_, 0.0);
  nh_private_.param<float>("car_Lon4_", car_Lon4_, 0.0);
  nh_private_.param<float>("car_Lat5_", car_Lat5_, 0.0);
  nh_private_.param<float>("car_Lon5_", car_Lon5_, 0.0);
  nh_private_.param<float>("car_Lat6_", car_Lat6_, 0.0);
  nh_private_.param<float>("car_Lon6_", car_Lon6_, 0.0);
  nh_private_.param<float>("car_Lat7_", car_Lat7_, 0.0);
  nh_private_.param<float>("car_Lon7_", car_Lon7_, 0.0);
  nh_private_.param<double>("Mission_sec", Mission_sec_, 200.0);
  nh_private_.param<double>("lost_sec_thre",lost_sec_thre_, 7.0);
  nh_private_.param<double>("v_thres_yawrotate",v_thres_yawrotate_, 1.7);
  nh_private_.param<bool>("auto_arm",auto_arm_, false);
  nh_private_.param<bool>("en_preset_mode",en_preset_mode_, false);
  nh_private_.param<bool>("en_yaw_rotate",en_yaw_rotate_, false);

  cout << "loop_sec: " << loop_sec_ << endl;
  cout << "Mission_sec: " << Mission_sec_ << endl;
  cout << "auto_arm: " << auto_arm_ << endl;
  cout << "lost_sec_thre: " << lost_sec_thre_ << endl;
  cout << "en_preset_mode: " << en_preset_mode_ << endl;
  cout << "v_thres_yawrotate: " << v_thres_yawrotate_ << endl;
  cout << "arrive_alt: " << arrive_alt_ << endl;
  cout << "track_alt: " << track_alt_ << endl;
  cout << "hover_yaw_rad: " << hover_yaw_ << endl;
  cout << "vxy_max: " << vxy_max_ << endl;
  cout << "acc_max: " << acc_max_ << endl;
  cout << "Kp: " << Kp_ << endl;
  cout << "Kd: " << Kd_ << endl;
  cout << "Ki: " << Ki_ << endl;
  cout << "car_initLat_deg: " << car_initLat_ << endl;
  cout << "car_initLon_deg: " << car_initLon_ << endl;
  cout << "en_yaw_rotate: " << en_yaw_rotate_ << endl;

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
  VxyPz_sp_tmp_ << 0.0,0.0,track_alt_;

  yaw_sp_ = hover_yaw_;

  gim_yaw_ = 0.0;
  gim_pitch_ = 0.0;
  gim_servo_state_ = "init";
  gim_track_state_ = "init";

  flag = 0;
  lost_num_ = 0;

  controller_state = PREPARE;
  last_state = PREPARE;
  command_ = BLANK;
  target_err_pE_ = 0;
  target_err_pN_ = 0;
  heading_ = 0;
  v_heading_ = 0;

  car_initposx_ = 0; car_initposy_ = 0;
  car_posx1_=0, car_posy1_=0, car_posx2_=0, car_posy2_=0, car_posx3_=0, car_posy3_=0;
  car_posx4_=0, car_posy4_=0, car_posx5_=0, car_posy5_=0, car_posx6_=0, car_posy6_=0;
  car_posx7_=0, car_posy7_=0;

  mavLat_ = 0;
  mavLon_ = 0;
  mavLat_init_ = 0;
  mavLon_init_ = 0;
  mavPE_init_ = 0;
  mavPN_init_ = 0; 

  time_init_ = 0;
  time_now_ = 0;
}

void uavCtrl::cmdloop_cb(const ros::TimerEvent &event)
{
  switch (controller_state)
  {
  case PREPARE: //waiting and initilizing
	//TODO: decide whether is right lat and lon
    mavLat_init_ = mavLat_;
    mavLon_init_ = mavLon_;
    mavPE_init_ = mavPos_(0);
    mavPN_init_ = mavPos_(1); 
    if(command_ != BLANK)
    {
      cout << "warning: the jc2fk.txt content is not wait!" << endl;
    }
    //TODO:how to avoid that the initial data in txt is takeoff
    if (command_ == LAUNCH)
    {
      time_init_ = ros::Time::now().toSec();
      controller_state = TAKEOFF;
      cout << "TAKEOFF" << endl;
      //controller_state = TRACK; //just for test,straight to track mode
      //cout << "TRACK" << endl; //just for test,straight to track mode
      break;
    }
    //compute car_initposx_, car_initposy_ in local ENU
    if ((car_initLat_ == 0) || (car_initLon_ == 0))
    {
      cout << "warning: car_initLat_ or car_initLon_ is 0!" << endl;
      break;
    }
    else
    {
      compute_preset_pos(); //gps to local
    }
    last_state = PREPARE;
    cout << "PREPARE" << endl;
    break;
  
  case TAKEOFF:
    if (command_ == BLANK)
    {
      controller_state = PREPARE;
      cout << "PREPARE" << endl;
      break;
    }
    if ((car_initposx_ == 0) || (car_initposy_ == 0))
    {
	cout << "warning: the jc2fk.txt content is not wait, break!" << endl;
        break;
    }
    if(!px4_state_.armed && auto_arm_)  //进入offboard自动解锁，危险，还是手动解锁吧
    {
      arm_cmd_.request.value = true; //spx
      arming_client_.call(arm_cmd_); //spx
      ros::Duration(0.5).sleep(); //spx
      cout << "not arming" <<endl;
    }
    //under 2m, continuing to trigger takeoff
    if ((px4_state_.mode != "AUTO.TAKEOFF")&&(mavPos_(2) < 2.0))
    {
      mode_cmd_.request.custom_mode = "AUTO.TAKEOFF";
      setMode_client_.call(mode_cmd_);
    }
    if ((px4_state_.mode != "AUTO.TAKEOFF")&&(mavPos_(2) > 2.0))
    {
      controller_state = FLYTOCAR;
      cout << "FLYTOCAR" << endl;
      break;
    }  
    if ((ros::Time::now().toSec()-time_init_) > Mission_sec_)
    {
      controller_state = FINISH;
      cout << "RETURN, time is up" << endl;
      break;      
    }
    last_state = TAKEOFF;
    //cout << "TAKEOFF" << endl;
    break;

  case FLYTOCAR:
    PxyPz_sp << car_initposx_, car_initposy_, arrive_alt_;
    pubPxyPzCmd(PxyPz_sp);
    if((px4_state_.mode != "OFFBOARD")&&(offboard_triggered_ == false)) //can switch to manual mode
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
      break;
    }
    if ((ros::Time::now().toSec()-time_init_) > Mission_sec_)
    {
      controller_state = FINISH;
      cout << "RETURN, time is up" << endl;
      break;      
    }
    last_state = FLYTOCAR;
    //cout << "FLYTOCAR" << endl;
    break;

  case HOVER_ON_CAR:
    PxyPz_sp << car_initposx_, car_initposy_, track_alt_;
    pubPxyzYawCmd(PxyPz_sp,hover_yaw_);
    if((fabs(mavPos_(2)-track_alt_) < 0.2)&&(fabs(heading_-hover_yaw_)<0.09))
    {
      controller_state = TRACK;
      cout << "TRACK" << endl;
      break;
    }
    if ((ros::Time::now().toSec()-time_init_) > Mission_sec_)
    {
      controller_state = FINISH;
      cout << "RETURN, time is up" << endl;
      break;      
    }
    last_state = HOVER_ON_CAR;
    //cout << "HOVER_ON_CAR" << endl;
    break;

  case TRACK:
    cout<<"TRACK"<<endl;
    if(px4_state_.mode != "OFFBOARD")
    {
      pidx.integral = 0.0;
      pidy.integral = 0.0;
      VxyPz_sp_ << 0.0,0.0,track_alt_;
      cout<<"WAITING OFFBOARD!"<<endl;
    }
    else //offboard
    {
      if(flag == 0)
      {
        if((lost_num_ < lost_sec_thre_/loop_sec_))
        {
          lost_num_ = lost_num_ +1;
          cout << "LOST! seconds: "<< lost_num_*loop_sec_ << endl;
        }
        else
        {
          lost_num_ = 0;
          //TODO: fly to a preset position, another mode
          if(en_preset_mode_)
          {
            controller_state = PRESET;
            cout << "FLY TO PRESET" << endl;
          }
        }
      }
      else
      {
        lost_num_ = 0;
        computeVelCmd(VxyPz_sp_, target_err_pE_, target_err_pN_);
        cout << "error_pE: "<<target_err_pE_<<", error_pN: "<<target_err_pN_<< endl;
        if(en_yaw_rotate_)
        {
		if(sqrt(mavVel_(0)*mavVel_(0)+mavVel_(1)*mavVel_(1)) > v_thres_yawrotate_)
		{
		  yaw_sp_ = v_heading_ + PI/2;
		//TODO: cancle yaw_sp change, pub v_heading_ to /yaw_east_to_head_deg
		}          
        }
      }
    }
    AccLimit(VxyPz_sp_);
    pubVxyPzYawCmd(VxyPz_sp_, yaw_sp_);
    //pubVxyPzCmd(VxyPz_sp_);

    if (command_ == RETURN)
    {
      controller_state = FINISH;
      cout << "RETURN, receive return command" << endl;
      break;
    }
    if ((ros::Time::now().toSec()-time_init_) > Mission_sec_)
    {
      controller_state = FINISH;
      cout << "RETURN, time is up" << endl;
      break;      
    }
    last_state = TRACK;
    //cout << "TRACK" << endl;
    break;

  case PRESET:
    pidx.integral = 0.0;
    pidy.integral = 0.0;
    VxyPz_sp_ << 0.0,0.0,track_alt_;
    if (flag == 1)
    {
      controller_state = TRACK;
      cout << "TRACK" << endl;
      break;
    }
    if (command_ == RETURN)
    {
      controller_state = FINISH;
      cout << "RETURN, receive return command" << endl;
      break;
    }
    if ((ros::Time::now().toSec()-time_init_) > Mission_sec_)
    {
      controller_state = FINISH;
      cout << "RETURN, time is up" << endl;
      break;      
    }
    if ((ros::Time::now().toSec()-time_init_) < (180 + 24))
    {
      PxyPz_sp << car_posx1_, car_posy1_, track_alt_;
      yaw_sp_ = -163*PI/180 + PI/2;
    }
    if ( ((ros::Time::now().toSec()-time_init_) >= (180 + 24)) &&
            ((ros::Time::now().toSec()-time_init_) < (180 + 49)) )
    {
      PxyPz_sp << car_posx2_, car_posy2_, track_alt_;
      yaw_sp_ = -76*PI/180 + PI/2;
    }
    if ( ((ros::Time::now().toSec()-time_init_) >= (180 + 49)) &&
            ((ros::Time::now().toSec()-time_init_) < (180 + 61)) )
    {
      PxyPz_sp << car_posx3_, car_posy3_, track_alt_;
      yaw_sp_ = 0*PI/180 + PI/2;
    }
    if ( ((ros::Time::now().toSec()-time_init_) >= (180 + 61)) &&
            ((ros::Time::now().toSec()-time_init_) < (180 + 84)) )
    {
      PxyPz_sp << car_posx4_, car_posy4_, track_alt_;
      yaw_sp_ = -87*PI/180 + PI/2;
    }
    if ( ((ros::Time::now().toSec()-time_init_) >= (180 + 84)) &&
            ((ros::Time::now().toSec()-time_init_) < (180 + 102)) )
    {
      PxyPz_sp << car_posx5_, car_posy5_, track_alt_;
      yaw_sp_ = -12*PI/180 + PI/2;
    }
    if ( ((ros::Time::now().toSec()-time_init_) >= (180 + 102)) &&
            ((ros::Time::now().toSec()-time_init_) < (180 + 125)) )
    {
      PxyPz_sp << car_posx6_, car_posy6_, track_alt_;
      yaw_sp_ = 20*PI/180 + PI/2;
    }
    if ( ((ros::Time::now().toSec()-time_init_) >= (180 +125)) &&
            ((ros::Time::now().toSec()-time_init_) < (180 + 146)) )
    {
      PxyPz_sp << car_posx7_, car_posy7_, track_alt_;
      yaw_sp_ = 143*PI/180 + PI/2;
    }
    if ( ((ros::Time::now().toSec()-time_init_) >= (180 + 146)) &&
            ((ros::Time::now().toSec()-time_init_) < (180 + 178)) )
    {
      PxyPz_sp << car_initposx_, car_initposy_, track_alt_;
    }

    pubPxyzYawCmd(PxyPz_sp,yaw_sp_);
    last_state = PRESET;
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
    cout << "RETURN, receive return command" << endl;
    break;

  default:
    cout << "uav controller: error controller state!" << endl;
    break;
  }
}

void uavCtrl::publoop_cb(const ros::TimerEvent &event)
{
  std_msgs::Float32 msg;
  msg.data = heading_*180/PI - 90;
  shape_yaw_pub_.publish(msg);
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
  //cout << "vx_cmd: " << vxypz_sp(0) << endl;

  pidy.error = error_py;
  pidy.derivative = error_py - pidy.error_last;
  pidy.integral += error_py;
  pidy.integral = max(-vxy_max_*0.8, min(pidy.integral, vxy_max_*0.8));//saturation resist
  pidy.error_last = error_py;
  vxypz_sp(1) = Kp_*pidy.error + Kd_*pidy.derivative + Ki_*pidy.integral;
  vxypz_sp(1) = max(-vxy_max_, min(vxy_max_, vxypz_sp(1)));
  //cout << "vy_cmd: " << vxypz_sp(1) << endl;

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
  error_pE_ = height/fabs(campose_ENU(2))*campose_ENU(0);
  error_pN_ = height/fabs(campose_ENU(2))*campose_ENU(1);
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

void uavCtrl::AccLimit(Eigen::Vector3d &cmd_sp)
{
  float dvx = cmd_sp(0) - mavVel_(0);
  float dvy = cmd_sp(1) - mavVel_(1);
//TODO: not minus mavVel_, but cmd_sp_last!
  float dv = pow((dvx*dvx+dvy*dvy),0.5) ;
  if(dv/loop_sec_>acc_max_)
  {
    cmd_sp(0) = mavVel_(0) + acc_max_*loop_sec_/dv*dvx;
    cmd_sp(1) = mavVel_(1) + acc_max_*loop_sec_/dv*dvy;
  }
}

void uavCtrl::pub_body_VxyPzCmd(const Eigen::Vector3d &cmd_sp)
{
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.coordinate_frame = 8; //pub in body local frame;
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
  mavAtt_(0) = msg.pose.orientation.x;
  mavAtt_(1) = msg.pose.orientation.y;
  mavAtt_(2) = msg.pose.orientation.z;
  mavAtt_(3) = msg.pose.orientation.w;//顺序
  Eigen::Quaterniond quaternion(mavAtt_);
  Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(2,1,0);
  if (abs(eulerAngle(1)) > 3) //pitch should be within (-pi/2,pi/2)
    heading_ = eulerAngle(0) - M_PI; //rad
  else
    heading_ = eulerAngle(0); //rad
    //cout<<heading_<<endl;
}

void uavCtrl::mavtwist_cb(const geometry_msgs::TwistStamped &msg)
{
  mavVel_(0) = msg.twist.linear.x;
  mavVel_(1) = msg.twist.linear.y;
  mavVel_(2) = msg.twist.linear.z;
  if (sqrt(mavVel_(0)*mavVel_(0)+mavVel_(1)*mavVel_(1)) > 0.02)
  {
    v_heading_ = atan2(mavVel_(1),mavVel_(0)); //rad
  }
}

void uavCtrl::globalpos_cb(const sensor_msgs::NavSatFix &msg)
{
  mavLat_ = msg.latitude;
  mavLon_ = msg.longitude;
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

void uavCtrl::targetCallback(const geometry_msgs::Quaternion&msg)
{
	//cout<<msg<<endl;
  float err_f;
  float err_l;
  err_f = tan(msg.x/57.3) * mavPos_(2);//msg.x is f angle
	err_l = tan(msg.y/57.3) * mavPos_(2);//y
  target_err_pE_ = err_f *cos(heading_)-err_l*sin(heading_);//east error
  target_err_pN_ = err_f*sin(heading_)+err_l*cos(heading_);
	flag = msg.w;
}

void uavCtrl::px4state_cb(const mavros_msgs::State &msg)
{
	px4_state_ = msg;
}

void uavCtrl::gps_to_local(float &local_E, float &local_N,
                               const float &target_lat, const float &target_lon,
                                const float &ref_E, const float &ref_N,
                                  const float &ref_lat, const float &ref_lon)
{
  std::array<double, 2> delta_EN;
  delta_EN = wgs84::toCartesian({ref_lat, ref_lon}, {target_lat, target_lon});
  local_E = ref_E + delta_EN[0];
  local_N = ref_N + delta_EN[1];
}

void uavCtrl::compute_preset_pos()
{
  gps_to_local(car_initposx_, car_initposy_,
                car_initLat_, car_initLon_,
                  mavPE_init_, mavPN_init_,
                    mavLat_init_, mavLon_init_);
  gps_to_local(car_posx1_, car_posy1_,
                car_Lat1_, car_Lon1_,
                  mavPE_init_, mavPN_init_,
                    mavLat_init_, mavLon_init_);  
  gps_to_local(car_posx2_, car_posy2_,
                car_Lat2_, car_Lon2_,
                  mavPE_init_, mavPN_init_,
                    mavLat_init_, mavLon_init_);         \
  gps_to_local(car_posx3_, car_posy3_,
                car_Lat3_, car_Lon3_,
                  mavPE_init_, mavPN_init_,
                    mavLat_init_, mavLon_init_);  
  gps_to_local(car_posx4_, car_posy4_,
                car_Lat4_, car_Lon4_,
                  mavPE_init_, mavPN_init_,
                    mavLat_init_, mavLon_init_);      
  gps_to_local(car_posx5_, car_posy5_,
                car_Lat5_, car_Lon5_,
                  mavPE_init_, mavPN_init_,
                    mavLat_init_, mavLon_init_);  
  gps_to_local(car_posx6_, car_posy6_,
                car_Lat6_, car_Lon6_,
                  mavPE_init_, mavPN_init_,
                    mavLat_init_, mavLon_init_);
  gps_to_local(car_posx7_, car_posy7_,
                car_Lat7_, car_Lon7_,
                  mavPE_init_, mavPN_init_,
                    mavLat_init_, mavLon_init_);                                      
}

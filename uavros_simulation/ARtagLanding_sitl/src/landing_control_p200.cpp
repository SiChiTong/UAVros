/***************************************************************************************************************************
*
* Author: shupx
* Email: shupx@buaa.edu.cn
* Time: 2021.1.12
* Description: 实现px4 quadrotor 二维码精准着陆，
***************************************************************************************************************************/
#include "landing_control_p200.h"
using namespace std;
using namespace Eigen;

PX4Landing::PX4Landing(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {
  Initialize();
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.05), &PX4Landing::CmdLoopCallback, this); //周期为 //0.05s spx
  //订阅降落板相对飞机位置
  ar_pose_sub_ = nh_.subscribe("ar_pose_marker", 1, &PX4Landing::ArPoseCallback, this,ros::TransportHints().tcpNoDelay());

  position_sub_ = nh_.subscribe("mavros/local_position/pose", 1, &PX4Landing::Px4PosCallback,this,ros::TransportHints().tcpNoDelay());

  rel_alt_sub_ = nh_.subscribe("mavros/global_position/rel_alt", 1, &PX4Landing::Px4RelAltCallback,this,ros::TransportHints().tcpNoDelay());

  state_sub_ = nh_.subscribe("mavros/state", 1, &PX4Landing::Px4StateCallback,this,ros::TransportHints().tcpNoDelay());

  cmd_sub = nh_.subscribe("command",10,&PX4Landing::cmdCallback,this,ros::TransportHints().tcpNoDelay()); 
  // 【服务】修改系统模式
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // 【服务】解锁/上锁 spx  include文件中还要定义arming_client和arm_cmd！
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
}

PX4Landing::~PX4Landing() {
  //Destructor
}

Eigen::Vector4d PX4Landing::LandingPidProcess(Eigen::Vector3d &currentPos,float currentYaw,Eigen::Vector3d &expectPos,float expectYaw)
{
  Eigen::Vector4d s_PidOut;

	/*X方向的pid控制*/
	s_PidItemX.difference = expectPos[0] - currentPos[0];
	s_PidItemX.intergral += s_PidItemX.difference;
	if(s_PidItemX.intergral >= 1)		
		s_PidItemX.intergral = 1;
	else if(s_PidItemX.intergral <= -1) 
		s_PidItemX.intergral = -1;
	s_PidItemX.differential =  s_PidItemX.difference  - s_PidItemX.tempDiffer;
  s_PidItemX.tempDiffer = s_PidItemX.difference;
//	cout << "s_PidItemX.tempDiffer: " << s_PidItemX.tempDiffer << endl;
//	cout << "s_PidItemX.differential: " << s_PidItemX.differential << endl;

	s_PidOut[0] = s_PidXY.p*s_PidItemX.difference + s_PidXY.d*s_PidItemX.differential + s_PidXY.i*s_PidItemX.intergral;
	if(s_PidOut[0] >= 1)		
		s_PidOut[0] = 1;
	else if(s_PidOut[0] <= -1) 
		s_PidOut[0] = -1;

	/*Y方向的pid控制*/
	s_PidItemY.difference = expectPos[1] - currentPos[1];
	s_PidItemY.intergral += s_PidItemY.difference;
	if(s_PidItemY.intergral >= 1)		
		s_PidItemY.intergral = 1;
	else if(s_PidItemY.intergral <= -1) 
		s_PidItemY.intergral = -1;
	s_PidItemY.differential =  s_PidItemY.difference  - s_PidItemY.tempDiffer;
  s_PidItemY.tempDiffer = s_PidItemY.difference;
	s_PidOut[1] = s_PidXY.p*s_PidItemY.difference + s_PidXY.d*s_PidItemY.differential + s_PidXY.i*s_PidItemY.intergral;
	if(s_PidOut[1] >= 1)		
		s_PidOut[1] = 1;
	else if(s_PidOut[1] <= -1) 
		s_PidOut[1] = -1;

	/*Z方向的pid控制*/
	s_PidItemZ.difference = expectPos[2] - currentPos[2];
	//s_PidItemZ.difference = expectPos[2] - px4_pose_[2]; //use px4 height rather than inaccurate ARtag height
	s_PidItemZ.intergral += s_PidItemZ.difference;
	if(s_PidItemZ.intergral >= 0.5)		
		s_PidItemZ.intergral = 0.5;
	else if(s_PidItemZ.intergral <= -0.5) 
		s_PidItemZ.intergral = -0.5;
	s_PidItemZ.differential =  s_PidItemZ.difference  - s_PidItemZ.tempDiffer;
  s_PidItemZ.tempDiffer = s_PidItemZ.difference;
	s_PidOut[2] = s_PidZ.p*s_PidItemZ.difference + s_PidZ.d*s_PidItemZ.differential + s_PidZ.i*s_PidItemZ.intergral;
	if(s_PidOut[2] >= 1)		
		s_PidOut[2] = 1;
	else if(s_PidOut[2] <= -0.2) 
		s_PidOut[2] = -0.2;

	/*Yaw方向的pid控制*/
	s_PidItemYaw.difference =  expectYaw - currentYaw;
	s_PidItemYaw.intergral += s_PidItemYaw.difference;
	if(s_PidItemYaw.intergral >= 0.3)		
		s_PidItemYaw.intergral = 0.3;
	else if(s_PidItemYaw.intergral <= -0.3) 
		s_PidItemYaw.intergral = -0.3;
	s_PidItemYaw.differential =  s_PidItemYaw.difference  - s_PidItemYaw.tempDiffer;
  s_PidItemYaw.tempDiffer = s_PidItemYaw.difference;
	s_PidOut[3] = s_PidYaw.p*s_PidItemYaw.difference + s_PidYaw.d*s_PidItemYaw.differential + s_PidYaw.i*s_PidItemYaw.intergral;

	return s_PidOut;
}

void PX4Landing::CmdLoopCallback(const ros::TimerEvent& event)
{
  LandingStateUpdate();
}


void PX4Landing::LandingStateUpdate()
{
	switch(LandingState)
	{
		case WAITING:
			//cout << "ar_pose_big_[0]:"  << ar_pose_big_[0] << endl;
			/*
			if(detect_state == true && px4_state_.mode == "AUTO.RTL")
			{
				detect_count++;
				cout << "detect goal:" << detect_count << endl;
			}
			if(detect_count == Thres_count_detect)
			{
				detect_count = 0; 
				mode_cmd_.request.custom_mode = "OFFBOARD";
				set_mode_client_.call(mode_cmd_);
			}
			*/
			approach_count = 0;
			small_count = 0;
			land_count = 0;
			command.data = 0; //reset command and counter
			if(px4_state_.mode != "OFFBOARD")//等待offboard模式
			{
				temp_pos_drone[0] = px4_pose_[0];
				temp_pos_drone[1] = px4_pose_[1];
				temp_pos_drone[2] = px4_pose_[2]; //spx 飞行按照气压计高度
				//temp_pos_drone[2] = rel_alt_;
				OffboardControl_.send_pos_xyz(temp_pos_drone); //即使没有进入offboard模式，也要不停发消息
				cout << "WAITING" <<endl; //spx
				// OffboardControl_.send_pos_setpoint(temp_pos_drone, 0);
			}
			if(px4_state_.mode == "OFFBOARD")
			{
				if(!px4_state_.armed)  //spx 进入offboard自动解锁，危险，还是手动解锁吧
				{
				     // arm_cmd_.request.value = true; //spx
				     // arming_client_.call(arm_cmd_); //spx
				     // //ros::Duration(0.5).sleep(); //spx
					cout << "not arming" <<endl;
				}
				
				temp_pos_drone[0] = px4_pose_[0];
				temp_pos_drone[1] = px4_pose_[1];
				temp_pos_drone[2] = px4_pose_[2]; //记录起飞前气压计初始高度
				//temp_pos_drone[2] = rel_alt_;

				LandingState = CHECKING;
				cout << "CHECKING" <<endl;
			}
			break;

		case CHECKING:
			if(px4_pose_[0] == 0 && px4_pose_[1] == 0) 			//没有位置信息则执行降落模式
			{   //检查gps给出的位置是不是正常的，默认精确的(0,0)是不正常值
				cout << "Check error, make sure have local location" <<endl;
				mode_cmd_.request.custom_mode = "AUTO.LAND";
				set_mode_client_.call(mode_cmd_);
				LandingState = WAITING;	
			}

			if((fabs(px4_pose_[0]-1) < 0.005) && (fabs(px4_pose_[1]-1) < 0.005)) 
			{	//检查uwb给出的位置是不是正常的，默认精确的(1,1)是不正常值，但是融合imu后会在(1,1)左右
				cout << "Check error, make sure the UWB local position is right" <<endl;
				mode_cmd_.request.custom_mode = "AUTO.LAND";
				set_mode_client_.call(mode_cmd_);
				LandingState = WAITING;	
			}

			if (temp_pos_drone[2]>0.2)  //spx 初始气压计高度太大了也不好
				{
				   temp_pos_drone[2] = 0.2;
				   cout << "High relative to ground" <<endl;
				   //mode_cmd_.request.custom_mode = "AUTO.LAND";
				   //set_mode_client_.call(mode_cmd_);
				   //LandingState = WAITING;	
				}
			if (temp_pos_drone[2]<-0.2) //spx 初始气压计高度太大了也不好
				{
				   temp_pos_drone[2] = -0.2;
				   cout << "High relative to ground" <<endl;
				   //mode_cmd_.request.custom_mode = "AUTO.LAND";
				   //set_mode_client_.call(mode_cmd_);
				   //LandingState = WAITING;	
				}
			else
			{
				LandingState = PREPARE;
				cout << "PREPARE" <<endl;
			}
			
			break;

		case PREPARE:	//起飞到指定高度
			//posxyz_target[0] = px4_pose_[0];  //时刻给当前的gps位置，保证x、y方向不动，实际上起飞过程会xy方向飘
			//posxyz_target[1] = px4_pose_[1]; //时刻给当前的gps位置，保证x、y方向不动，实际上起飞过程会xy方向飘
			//可以尝试OffboardControl::send_body_velxy_posz_setpoint,xy方向速度给0
			posxyz_target[0] = temp_pos_drone[0];  //时刻给起飞点位置（uwb保证精度）
			posxyz_target[1] = temp_pos_drone[1]; //时刻给起飞点位置（uwb保证精度）
			posxyz_target[2] = search_alt_; //spx在气压计初始高度的基础上飞search_alt_高，如果高度准就不需要加初直了
			OffboardControl_.send_pos_xyz(posxyz_target);

			if(command.data == 1)
			{
				if(fabs(px4_pose_[2]-search_alt_) < 0.2) // confirm the height first
				{
					LandingState = SEARCH;
					//LandingState = APPROACH;
				}
				else
				{
					cout << "Continue preparing" <<endl;
				}
			}				
			if(px4_state_.mode != "OFFBOARD" )		//如果在准备中途中切换到手动，则跳到WAITING
			{
				LandingState = WAITING;
				command.data = 0;
			}
			cout << "PREPARE" <<endl;
			break;

		case SEARCH:
			if(detect_big == true)
			{
				LandingState = APPROACH;
			  	cout << "APPROACH" <<endl;
			}	
			else//搜寻目标
			{
				posxyz_target[0] = temp_pos_drone[0];  //时刻给起飞点位置（uwb保证精度）;
				posxyz_target[1] = temp_pos_drone[1];  //时刻给起飞点位置（uwb保证精度）;
				posxyz_target[2] = search_alt_;
				desire_yawVel_ = 0.15; //rotate to find the ARtag 
				OffboardControl_.send_pos_yawrate_setpoint(posxyz_target, desire_yawVel_);
			}

			if(px4_state_.mode != "OFFBOARD")		//如果在SEARCH途中切换到手动，则跳到WAITING
			{
				LandingState = WAITING;
				command.data = 0;
			}
      		
			cout << "SEARCHING" <<endl;
			break;
		
		case APPROACH:
			{
				if(px4_state_.mode != "OFFBOARD")
				{
					LandingState = WAITING;
					command.data = 0;
				}				

				if(detect_big == true)
				{
					desire_vel_ = LandingPidProcess(ar_pose_big_,markers_yaw_,desire_pose_,desire_yaw_);
					//desire_vel_ = LandingPidProcess(ar_pose_big_,markers_yaw_,desire_pose_approach,desire_yaw_);
					//desire_vel_ = LandingPidProcess(ar_pose_small_,markers_yaw_,desire_pose_,desire_yaw_);

					desire_xyVel_[0] = desire_vel_[0];
					desire_xyVel_[1] = desire_vel_[1];
					desire_xyVel_[2] = 0;
					desire_yawVel_ = 0;
					cout << "APPROACH" <<endl;

					if((fabs(ar_pose_big_[0]) < 0.10) && (fabs(ar_pose_big_[1]) < 0.10)) //水平误差
					{
						approach_count ++;
						cout<< "approach_count: " << approach_count <<endl;						
					}
				}
			  	else
				{
					desire_xyVel_[0] = 0;
					desire_xyVel_[1] = 0;
					desire_xyVel_[2] = 0;
					desire_yawVel_ = 0;
					cout << "APPROACH:big missing" <<endl;
				}

				//OffboardControl_.send_body_velxyz_setpoint(desire_xyVel_,desire_yawVel_);
				OffboardControl_.send_body_velxy_posz_setpoint(desire_xyVel_, search_alt_);

				if((detect_state == true) && approach_count == Thres_count_approach)
				{
					LandingState = LANDING;
					approach_count = 0;
					cout << "enter LANDING" << endl;
				}				
			}
			break;
/*
		case ALIGN:
			{
				if(px4_state_.mode != "OFFBOARD")
				{
					LandingState = WAITING;
					command.data = 0;
				}				

				if(detect_big == true)
				{
					desire_vel_ = LandingPidProcess(ar_pose_big_,markers_yaw_,desire_pose_,desire_yaw_);

					desire_xyVel_[0] = desire_vel_[0];
					desire_xyVel_[1] = desire_vel_[1];
					desire_xyVel_[2] = 0;
					desire_yawVel_ = desire_vel_[3];
					cout << "Rotating" <<endl;
				}
			  	else
				{
					desire_xyVel_[0] = 0;
					desire_xyVel_[1] = 0;
					desire_xyVel_[2] = 0;
					desire_yawVel_ = 0;
					cout << "Rotating:big missing" <<endl;
				}

				//OffboardControl_.send_body_velxyz_setpoint(desire_xyVel_,desire_yawVel_);
				OffboardControl_.send_body_velxy_posz_yawrate_setpoint(desire_xyVel_, search_alt_, desire_yawVel_);

				if(detect_state == true && fabs(markers_yaw_ - desire_yaw_) < 0.15)
				{
					LandingState = LANDING;
					cout << "enter LANDING" << endl;
				}				
			}
			break;
*/
		case LANDING:
			{				
				if(!detect_state)
				{
					desire_vel_[0] = 0;
					desire_vel_[1] = 0;
					desire_vel_[2] = 0;
					desire_vel_[3] = 0;
				}
				if(detect_big == true)
				{
					desire_vel_ = LandingPidProcess(ar_pose_big_,markers_yaw_,desire_pose_,desire_yaw_);
				}

				if(detect_big == false && detect_small == true)
				{
					desire_vel_ = LandingPidProcess(ar_pose_small_,markers_yaw_,desire_pose_,desire_yaw_);
				}

				if(detect_small == true) 
				{
					small_count ++;					
					cout << "small count:" << small_count << endl;
				}

				if(detect_small == true && small_count > Thres_count_small && px4_pose_[2] < Thres_height_small)
				{
					LandingState = LANDING_SMALL;   
					small_count = 0;
					cout << "ARtag SMALL" <<endl;  
				}

				if(px4_state_.mode != "OFFBOARD")			//如果在LANDING中途中切换到onboard，则跳到WAITING
				{
					LandingState = WAITING;
					command.data = 0;
				}

				desire_xyVel_[0] = desire_vel_[0];
				desire_xyVel_[1] = desire_vel_[1];
				//desire_xyVel_[2] = desire_vel_[2]; //这里采用pid给出的z轴速度
				desire_xyVel_[2] = -landing_vel; //这里下降用恒速
				desire_yawVel_ = desire_vel_[3];
				OffboardControl_.send_body_velxyz_setpoint(desire_xyVel_,desire_yawVel_);
				cout << "LANDING--ARtag big" << endl;				
			}
			break;

		case LANDING_SMALL:
			{
				if(px4_state_.mode != "OFFBOARD")
				{
					LandingState = WAITING;
					command.data = 0;
				}	

				if(detect_small == true)
				{
					desire_vel_ = LandingPidProcess(ar_pose_small_,markers_yaw_,desire_pose_,desire_yaw_);
					desire_xyVel_[0] = desire_vel_[0]; 
					desire_xyVel_[1] = desire_vel_[1];  
					//desire_xyVel_[2] = desire_vel_[2]; //这里采用pid给出的z轴速度
					desire_xyVel_[2] = -scale_factor*landing_vel; //这里下降用恒速
					desire_yawVel_ = desire_vel_[3];
					cout << "ARtag SMALL" << endl;
				}
				if(detect_small == false)
				{	
					desire_xyVel_[0] = 0;
					desire_xyVel_[1] = 0;
					//desire_xyVel_[2] = desire_vel_[2]; //保持速度下降
					desire_xyVel_[2] = -scale_factor*landing_vel; //这里下降用恒速
					desire_yawVel_ = 0;
					cout << "ARtag SMALL:small missing" <<endl;
				}

				OffboardControl_.send_body_velxyz_setpoint(desire_xyVel_,desire_yawVel_);

				if(px4_pose_[2] <= landoverH_ ||ar_pose_small_[2] <= landoverH_tag_) //when landing lower than 0.5m ,distance sensor is not accurate at all.camera not calibrated well and ar_pose scale 3rd
				{
					//if(detect_small == true && (fabs(ar_pose_small_[0]) < landoverXY_) && (fabs(ar_pose_small_[1]) < landoverXY_))
					land_count ++;
					cout << "land count:" << land_count << endl;
				}
				if(land_count >= Thres_count_land)
				{
					LandingState = LANDOVER;
					land_count = 0;
					cout << "AUTO.LAND" << endl;
				}				
			}
			break;
		
		case LANDOVER:
			{
				mode_cmd_.request.custom_mode = "AUTO.LAND";
        		set_mode_client_.call(mode_cmd_);
				LandingState = LANDOVER;
				//LandingState = WAITING;  
				//降落后回到WAITING状态，可能又会起飞！
				//降落后pixhawk一直叫，需要关闭这个程序才不叫了，这个收尾还是要考虑一下
			}
			break;

		default:
			cout << "error state" <<endl;
	}	
}

/*接收降落板相对飞机的位置以及偏航角*/
void PX4Landing::ArPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
	detect_state = false;
	detect_small = false;
	detect_big = false;
  double temp_roll,temp_pitch,temp_yaw;
  tf::Quaternion quat;
	for(auto &item : msg->markers)
	{
		if(item.id == small_marker_id_)
		{
			detect_state = true;
			detect_small = true;
      		//ar_pose_small_[0] = -(item.pose.pose.position.x - 0.0); //old kinetic
      		//ar_pose_small_[1] = (item.pose.pose.position.y - 0.0);  //old kinetic
      		ar_pose_small_[0] = (item.pose.pose.position.y - 0.0); //melodic
      		ar_pose_small_[1] = (item.pose.pose.position.x - 0.0); //melodic
      		ar_pose_small_[2] = item.pose.pose.position.z;
      		tf::quaternionMsgToTF(item.pose.pose.orientation,quat);
      		tf::Matrix3x3(quat).getRPY(temp_roll,temp_pitch,temp_yaw);
			markers_yaw_ = temp_yaw;
//			cout << "ar_pose_[0]:"  << ar_pose_[0] << endl;
//			cout << "ar_pose_[1]:"  << ar_pose_[1] << endl;
//			cout << "ar_pose_[2]:"  << ar_pose_[2] << endl;
//			cout << "markers_yaw_: "  << markers_yaw_ << endl;
		}
		if(item.id == big_marker_id_)
		{
			detect_state = true;
			detect_big = true;
      		//ar_pose_big_[0] = -(item.pose.pose.position.x - 0.0) *big_marker_size/small_marker_size; //old kinetic
      		//ar_pose_big_[1] = (item.pose.pose.position.y - 0.0) *big_marker_size/small_marker_size; //old kinetic
      		ar_pose_big_[0] = (item.pose.pose.position.y - 0.0) *big_marker_size/small_marker_size; //melodic
      		ar_pose_big_[1] = (item.pose.pose.position.x - 0.0) *big_marker_size/small_marker_size; //melodic
      		ar_pose_big_[2] = item.pose.pose.position.z *big_marker_size/small_marker_size;
      		tf::quaternionMsgToTF(item.pose.pose.orientation,quat);
      		tf::Matrix3x3(quat).getRPY(temp_roll,temp_pitch,temp_yaw);
			markers_yaw_ = temp_yaw;
//			cout << "ar_pose_[0]:"  << ar_pose_[0] << endl;
//			cout << "ar_pose_[1]:"  << ar_pose_[1] << endl;
//			cout << "ar_pose_[2]:"  << ar_pose_[2] << endl;
//			cout << "markers_yaw_: "  << markers_yaw_ << endl;
		}
	}
//	cout << "detect_state :" << detect_state << endl;
}

/*接收来自飞控的当前飞机位置*/                  
void PX4Landing::Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    px4_pose_ = pos_drone_fcu_enu;
}

/*接受来自飞控的当前真实高度数据*/
void PX4Landing::Px4RelAltCallback(const std_msgs::Float64::ConstPtr& msg)
{
	double  data;
	data = msg->data;
	rel_alt_ = data;	
}

/*接收来自飞控的当前飞机状态*/
void PX4Landing::Px4StateCallback(const mavros_msgs::State::ConstPtr& msg)
{
	px4_state_ = *msg;
}

void PX4Landing::cmdCallback(const std_msgs::Int32::ConstPtr& msg)
{
	command = *msg;
	cout << "receive command: " << command.data << endl;
}


/*初始化*/
void PX4Landing::Initialize()
{
  //读取offboard模式下飞机的搜索高度
  nh_private_.param<float>("search_alt_", search_alt_, 0.1);
  //小二维码的id号,大二维码用作高处的水平面接近，不使用其高度数据。
  nh_private_.param<float>("big_marker_id", big_marker_id_, 4);
  nh_private_.param<float>("small_marker_id", small_marker_id_, 0);
  nh_private_.param<float>("PidXY_p", s_PidXY.p, 0.5);
  nh_private_.param<float>("PidXY_d", s_PidXY.d, 0);
  nh_private_.param<float>("PidXY_i", s_PidXY.i, 0);
  nh_private_.param<float>("PidZ_p", s_PidZ.p, 0.07);
  nh_private_.param<float>("PidZ_d", s_PidZ.d, 0);
  nh_private_.param<float>("PidZ_i", s_PidZ.i, 0);
  nh_private_.param<float>("PidYaw_p", s_PidYaw.p, 0.13);
  nh_private_.param<float>("PidYaw_d", s_PidYaw.d, 0);
  nh_private_.param<float>("PidYaw_i", s_PidYaw.i, 0);
  nh_private_.param<float>("big_marker_size", big_marker_size, 17.4);
  nh_private_.param<float>("small_marker_size", small_marker_size, 5.2);

  nh_private_.param<int>("Thres_count_detect",Thres_count_detect, 30);
  nh_private_.param<int>("Thres_count_approach",Thres_count_approach, 10);
  nh_private_.param<int>("Thres_count_small",Thres_count_small, 20);
  nh_private_.param<float>("Thres_height_small",Thres_height_small, 1);
  nh_private_.param<int>("Thres_count_land",Thres_count_land, 5);
  nh_private_.param<float>("landing_vel", landing_vel, 0.11);
  nh_private_.param<float>("scale_factor", scale_factor, 1);
  nh_private_.param<float>("landoverH", landoverH_, 0.2);
  nh_private_.param<float>("landoverH_tag", landoverH_tag_, 0.1);
  //nh_private_.param<float>("landoverXY", landoverXY_, 0.05);

  //期望的飞机相对降落板的位置
  float desire_pose_x,desire_pose_y,desire_pose_z;
  nh_private_.param<float>("desire_pose_x", desire_pose_x, 0);
  nh_private_.param<float>("desire_pose_y", desire_pose_y, 0);
  nh_private_.param<float>("desire_pose_z", desire_pose_z, 0);
  nh_private_.param<float>("desire_yaw_", desire_yaw_, 0);
  desire_pose_[0] = desire_pose_x;
  desire_pose_[1] = desire_pose_y;
  desire_pose_[2] = desire_pose_z;

  desire_pose_approach[0] = desire_pose_x;
  desire_pose_approach[1] = desire_pose_y;
  desire_pose_approach[2] = search_alt_;

  detect_state = false;
  detect_big = false;
  detect_small = false;
  desire_vel_[0] = 0;
  desire_vel_[1] = 0;
  desire_vel_[2] = 0;
  desire_vel_[3] = 0;
	desire_xyVel_[0]  = 0;
	desire_xyVel_[1]  = 0;
	desire_xyVel_[2]  = 0;
	desire_yawVel_ = 0;
  s_PidItemX.tempDiffer = 0;
  s_PidItemY.tempDiffer = 0;
  s_PidItemZ.tempDiffer = 0;
  s_PidItemYaw.tempDiffer = 0;
  s_PidItemX.intergral = 0;
  s_PidItemY.intergral = 0;
  s_PidItemZ.intergral = 0;
  s_PidItemYaw.intergral = 0;

  command.data = 0;

	cout << "search_alt_ = " << search_alt_ << endl;
	cout << "big_marker_id = " << big_marker_id_ << endl;
	cout << "small_marker_id = " << small_marker_id_ << endl;
	cout << "big_marker_size = " << big_marker_size << endl;
	cout << "small_marker_size = " << small_marker_size << endl;
	cout << "PidXY_p = " << s_PidXY.p << endl;
	cout << "PidXY_d = " << s_PidXY.d << endl;
	cout << "PidXY_i = " << s_PidXY.i << endl;
	cout << "PidZ_p = " << s_PidZ.p << endl;
	cout << "PidZ_d = " << s_PidZ.d << endl;
	cout << "PidZ_i = " << s_PidZ.i << endl;
	cout << "PidYaw_p = " << s_PidYaw.p << endl;
	cout << "PidYaw_d = " << s_PidYaw.d << endl;
	cout << "PidYaw_i = " << s_PidYaw.i << endl;
	cout << "desire_pose_x = " << desire_pose_[0] << endl;
	cout << "desire_pose_y = " << desire_pose_[1] << endl;
	cout << "desire_pose_z = " << desire_pose_[2] << endl;
	cout << "desire_yaw_ = " << desire_yaw_ << endl;
	cout << "Thres_count_detect = " << Thres_count_detect << endl;
	cout << "Thres_count_approach = " << Thres_count_approach << endl;
	cout << "Thres_count_small = " << Thres_count_small << endl;
	cout << "Thres_height_small = " << Thres_height_small << endl;
	cout << "Thres_count_land = " << Thres_count_land << endl;
	cout << "landing_vel = " << landing_vel << endl;
	cout << "scale_factor = " << scale_factor << endl;
	cout << "landoverH = " << landoverH_ << endl;
	cout << "landoverH_tag = " << landoverH_tag_ << endl;
	//cout << "landoverXY = " << landoverXY_ << endl;

}
int main(int argc, char** argv) {
  ros::init(argc,argv,"landing_control_p200");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  PX4Landing PX4Landing(nh, nh_private);

  ros::spin();
  return 0;
}
/**
 * @brief UAV controller class
 *
 * Flight controller to output velocity setpoint
 *
 * @author Peixuan Shu <shupeixuan@qq.com>
 * 
 * First built on 2021.07.10
 */

#ifndef _UAV_CONTROLLER_H
#define _UAV_CONTROLLER_H

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Core>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>
#include "uavros_msgs/TrackState.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/NavSatFix.h>
//#include <dynamic_reconfigure/server.h>
//#include <uavros_wrzf_sitl/dynamicConfig.h>
#include "WGS84toCartesian.hpp"

#define PI 3.1415926

using namespace std;
using namespace Eigen;

class uavCtrl 
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber mavposeSub_;
    ros::Subscriber mavtwistSub_;
    ros::Subscriber px4stateSub_;
    //ros::Subscriber gimbalSub_;
    ros::Subscriber jc_cmdSub_;
    ros::Subscriber globalposSub_;
    ros::Publisher target_pose_pub_; 
    //ros::Publisher en_track_pub_;   
    ros::Publisher shape_yaw_pub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient setMode_client_;
    ros::Timer publoop_timer_;
    ros::Timer cmdloop_timer_;
    ros::Subscriber target_err_Sub_;

    Eigen::Vector4d mavAtt_;
    Eigen::Vector3d mavPos_, mavVel_;
    Eigen::Vector3d PxyPz_sp;
    Eigen::Vector3d VxyPz_sp_;
    Eigen::Vector3d VxyPz_sp_tmp_;

    bool auto_arm_;
    bool en_preset_mode_;
    bool en_yaw_rotate_;
    double loop_sec_;
    double arrive_alt_, track_alt_;
    double Kp_, Kd_, Ki_;
    double vxy_max_;
    double acc_max_;
    double error_pE_, error_pN_;
    double hover_yaw_;
    double yaw_sp_;
    double time_init_, time_now_;
    double Mission_sec_;
    double v_thres_yawrotate_;

    int flag;
    int lost_num_;
    double lost_sec_thre_;

    float gim_yaw_;
    float gim_pitch_;
    string gim_servo_state_;
    string gim_track_state_;
    float target_err_pE_;
    float target_err_pN_;
    float heading_;
    float v_heading_;

    float mavLat_;
    float mavLon_;
    float mavLat_init_;
    float mavLon_init_;
    float mavPE_init_;
    float mavPN_init_; 
    float car_initposx_, car_initposy_;
    float car_posx1_, car_posy1_, car_posx2_, car_posy2_, car_posx3_, car_posy3_; 
    float car_posx4_, car_posy4_, car_posx5_, car_posy5_, car_posx6_, car_posy6_; 
    float car_posx7_, car_posy7_;
    float car_initLat_, car_initLon_;
    float car_Lat1_, car_Lon1_, car_Lat2_, car_Lon2_, car_Lat3_, car_Lon3_; 
    float car_Lat4_, car_Lon4_, car_Lat5_, car_Lon5_, car_Lat6_, car_Lon6_;
    float car_Lat7_, car_Lon7_;
  /*
    # STATE DICT
    servo_state_dict = {
        0x00: "CLOSED",
        0x41: "MANUAL",
        0x44: "POSITION",
        0x66: "TRACK"
    }

    track_state_dict = {
        0x00: "NoTarget",
        0x01: "HaveTarget",
        0x02: "Tracking",
        0x03: "Missing"
    }
  */
    
    bool takeoff_triggered_;
    bool offboard_triggered_;
    bool return_triggered_;

    enum ControllerState
    {
      PREPARE,
      TAKEOFF,
      FLYTOCAR,
      HOVER_ON_CAR,		
      TRACK,
      PRESET,
      FINISH
    };
    ControllerState controller_state;
    ControllerState last_state;

    enum Command
    { BLANK,
      LAUNCH,
      FLYUP,
      TO_ONE,
      TO_TWO,
      TO_THREE,		
      RETURN
    };
    Command command_;

    mavros_msgs::State px4_state_;
    mavros_msgs::SetMode mode_cmd_;
    mavros_msgs::CommandBool arm_cmd_;
    std_msgs::String string_msg_;
    //uavros_msgs::TrackState gimbal_state_;

    typedef struct
    {
      double error;  
      double error_last; 
      double derivative;	
      double integral;     
    } PID_ITEM;
    PID_ITEM pidx;
    PID_ITEM pidy;
    void cmdloop_cb(const ros::TimerEvent &event);
    void publoop_cb(const ros::TimerEvent &event);
    void mavpose_cb(const geometry_msgs::PoseStamped &msg);
    void mavtwist_cb(const geometry_msgs::TwistStamped &msg);
    void globalpos_cb(const sensor_msgs::NavSatFix &msg);

    void computeVelCmd(Eigen::Vector3d &vxypz_sp, const double &error_px, const double &error_py);
    void computeError(const float &yaw, const float &pitch,
              const Eigen::Vector4d &uavatt, const Eigen::Vector3d &uavpos);
    void pubPxyPzCmd(const Eigen::Vector3d &cmd_p);
    void pubPxyzYawCmd(const Eigen::Vector3d &cmd_p, const double &yaw_sp);
    void pubVxyPzYawCmd(const Eigen::Vector3d &cmd_sp, const double &yaw_sp);
    void pubVxyPzCmd(const Eigen::Vector3d &cmd_sp);
    
    void jc_cmd_cb(const std_msgs::Int32 &msg);
    void px4state_cb(const mavros_msgs::State &msg);
    //void gimbal_cb(const uavros_msgs::TrackState &msg);

    void targetCallback(const geometry_msgs::Quaternion&msg);
    void pub_body_VxyPzCmd(const Eigen::Vector3d &cmd_sp);
    void AccLimit(Eigen::Vector3d &cmd_sp);
    void gps_to_local(float &local_E, float &local_N,
                        const float &target_lat, const float &target_lon,
                          const float &ref_E, const float &ref_N,
                            const float &ref_lat, const float &ref_lon);
    void compute_preset_pos();

    //void leaderpose_cb(const mavros_msgs::PositionTarget &msg);
    //void cmd_cb(const std_msgs::Int32 &msg);

  public:
    //void dynamic_callback(const uavros_wrzf_sitl::dynamicConfig &config);
    uavCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    //virtual ~uavCtrl();
    
};

#endif

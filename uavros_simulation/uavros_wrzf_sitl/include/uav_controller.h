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
#include <std_srvs/SetBool.h>



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
    ros::Subscriber jc_cmdSub_;
    ros::Publisher target_pose_pub_; 
    ros::Publisher en_track_pub_;   
    ros::ServiceClient arming_client_;
    ros::ServiceClient setMode_client_;
    ros::Timer cmdloop_timer_;

    Eigen::Vector4d mavAtt_;
    Eigen::Vector3d mavPos_, mavVel_;
    Eigen::Vector3d PxyPz_sp;
    Eigen::Vector3d VxyPz_sp_;

    double arrive_alt_, track_alt_;
    double Kp_, Kd_, Ki_;
    double vxy_max_;
    double error_pE, error_pN;
    double car_initposx_, car_initposy_;
    double hover_yaw_;
    double yaw_sp_;
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
    std_msgs::String string_msg_;

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
    void mavpose_cb(const geometry_msgs::PoseStamped &msg);
    void mavtwist_cb(const geometry_msgs::TwistStamped &msg);

    void computeVelCmd(Eigen::Vector3d &vxypz_sp, const double &error_px, const double &error_py);
    void pubPxyPzCmd(const Eigen::Vector3d &cmd_p);
    void pubPxyzYawCmd(const Eigen::Vector3d &cmd_p, const double &yaw_sp);
    void pubVxyPzYawCmd(const Eigen::Vector3d &cmd_sp, const double &yaw_sp);
    
    void jc_cmd_cb(const std_msgs::Int32 &msg);
    void px4state_cb(const mavros_msgs::State &msg);

    //void leaderpose_cb(const mavros_msgs::PositionTarget &msg);
    //void cmd_cb(const std_msgs::Int32 &msg);

  public:
    uavCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    
};

#endif
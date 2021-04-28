/**
 * @brief UAV controller class
 *
 * UAV controller to output acceleration setpoint
 *
 * @author Peixuan Shu <shupeixuan@qq.com>
 * 
 * First built on 2021.04.02
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

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int32.h>



using namespace std;
using namespace Eigen;

class uavCtrl 
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber mavposeSub_;
    ros::Subscriber mavtwistSub_;
    ros::Subscriber leaderposeSub_;
    ros::Subscriber cmdSub_;
    ros::Subscriber px4stateSub_;
    ros::Publisher target_pose_pub_;    
    ros::ServiceClient arming_client_;
    ros::ServiceClient setMode_client_;
    ros::Timer cmdloop_timer_;

    Eigen::Vector4d mavAtt_;
    Eigen::Vector3d mavPos_, mavVel_, leaderPos_, leaderVel_, leaderAcc_;
    Eigen::Vector3d acc_sp, VxyPz_sp;
    Eigen::Vector3d hPos_, hVel_, hAcc_;
    Eigen::Vector3d Kpos_, Kvel_;
   
    double alt_sp;
    double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_;
    double init_x_, init_y_;
    double t1_start_, t1_accum_, t_;
    double h_omega_, h_radius_, h_phi_;
    int command_, state_num_;
    mavros_msgs::State px4_state_;
    mavros_msgs::SetMode mode_cmd_;

    void cmdloop_cb(const ros::TimerEvent &event);
    void mavpose_cb(const geometry_msgs::PoseStamped &msg);
    void mavtwist_cb(const geometry_msgs::TwistStamped &msg);
    void leaderpose_cb(const mavros_msgs::PositionTarget &msg);
    void computeAccCmd(Eigen::Vector3d &acc_cmd, const Eigen::Vector3d &target_pos,
                                       const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc);
    void pubAccCmd(const Eigen::Vector3d &cmd_acc);
    void pubVxyPzCmd(const Eigen::Vector3d &cmd_sp);
    void cmd_cb(const std_msgs::Int32 &msg);
    void px4state_cb(const mavros_msgs::State &msg);

  public:
    uavCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    
};

#endif
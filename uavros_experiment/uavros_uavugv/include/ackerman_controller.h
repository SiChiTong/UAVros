/**
 * @brief ackerman controller class
 *
 * ackerman controller to output v and delta setpoint
 *
 * @author Peixuan Shu <shupeixuan@qq.com>
 * 
 * First built on 2021.04.02
 */

#ifndef _ACKERMAN_CONTROLLER_H
#define _ACKERMAN_CONTROLLER_H

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>


using namespace std;
using namespace Eigen;

class ackermanCtrl 
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber ugvposeSub_;
    ros::Subscriber leaderposeSub_;
    ros::Subscriber cmdSub_;
    ros::Publisher target_pose_pub_;    
    ros::Timer cmdloop_timer_;

    Eigen::Vector4d ugvAtt_;
    Eigen::Vector3d ugvPos_, ugvVel_, leaderPos_, leaderVel_, leaderAcc_;
    Eigen::Vector3d Kpos_;
   
    double v_max_, delta_max_, heading_, L_;
    double Kpos_x_, Kpos_y_;
    double start_time_, t_;
    double v_sp_, delta_sp_, beta_;
    int command_, start_flag_;


    void cmdloop_cb(const ros::TimerEvent &event);
    void ugvpose_cb(const nav_msgs::Odometry &msg);

    void cmd_cb(const std_msgs::Int32 &msg);
    void leaderpose_cb(const mavros_msgs::PositionTarget &msg);
    void computeSetpoint(double &v_cmd, double &delta_cmd, const Eigen::Vector3d &target_pos,
                        const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc);
    void pubSetpoint(const double &v_sp_, const double &delta_sp_);

  public:
    ackermanCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    
};

#endif

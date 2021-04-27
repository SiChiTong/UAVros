/**
 * @brief leader estimator class
 *
 * leader state estimator
 *
 * @author Peixuan Shu <shupeixuan@qq.com>
 * 
 * First built on 2021.04.04
 */

#ifndef _LEADER_ESTIMATOR_H
#define _LEADER_ESTIMATOR_H

#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>


using namespace std;
using namespace Eigen;

class leaderEstimate 
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber ng_estimateSub1_;
    ros::Subscriber cmdSub_;
    ros::Publisher leader_state_pub_;   
    ros::Publisher leaderPath_pub_; 
    ros::Timer cmdloop_timer_;
    ros::Timer trajloop_timer_;

    Eigen::Vector3d leaderPos_, leaderVel_, leaderAcc_;
    Eigen::Vector4d q_hat_; //estimation of leader state, pos and vel
    Eigen::Vector4d q_hat_last_; //estimation of leader state in the last loop, pos and vel
    Eigen::Vector2d y_hat1_; //neighbor_1's estimation of leader output, pos
    Eigen::Vector2d y_hat2_; //neighbor_2's estimation of leader output, pos
    Eigen::Matrix<double,4,4> A0_;
    Eigen::Matrix<double,4,2> KP_; // gain matrix of the distributed estimator

    double shape_omega_, shape_radius_;
    double dt_, time_now_, time_last_, t1_start_, t1_accum_, t_;
    double alt_sp, alpha_;
    double esm_x_init_, esm_y_init_;
    double l_vx_, l_vy_;
    string ng_name1_, ng_name2_;
    int ng_num_;
    int command_, state_num_;
    int trajpose_window_;
    std::vector<geometry_msgs::PoseStamped> TrajPoseHistory_vector_;

    void cmdloop_cb(const ros::TimerEvent &event);
    void trajloop_cb(const ros::TimerEvent &event);
    void circleCreator(double t);
    void pubLeaderEstimation(const Eigen::Vector3d &state_pos, const Eigen::Vector3d &state_vel, const Eigen::Vector3d &state_acc);
    void pubTrajectory();
    void distributed_estimator(double dt);
    void ng_estimate_cb1(const mavros_msgs::PositionTarget &msg);
    void cmd_cb(const std_msgs::Int32 &msg);

  public:
    leaderEstimate(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    
};

#endif
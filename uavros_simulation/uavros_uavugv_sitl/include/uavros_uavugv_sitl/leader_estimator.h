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


using namespace std;
using namespace Eigen;

class leaderEstimate 
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher leader_state_pub_;   
    ros::Publisher leaderPath_pub_; 
    ros::Timer cmdloop_timer_;
    ros::Timer trajloop_timer_;

    Eigen::Vector3d leaderPos_, leaderVel_, leaderAcc_;
   
    double shape_omega_, shape_radius_;
    double dt_, time_now_, time_last_;
    double alt_sp;
    geometry_msgs::PoseStamped trajPoseStamped[70]; //rviz Path: last few seconds, 0.1s/point

    void cmdloop_cb(const ros::TimerEvent &event);
    void trajloop_cb(const ros::TimerEvent &event);
    void shapeCreator(double t);
    void pubLeaderEstimation(const Eigen::Vector3d &state_pos, const Eigen::Vector3d &state_vel, const Eigen::Vector3d &state_acc);
    void pubTrajectory();

  public:
    leaderEstimate(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    
};

#endif
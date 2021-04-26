/**
 * @brief leader estimator node
 *
 * leader state estimator
 *
 * @author Peixuan Shu <shupeixuan@qq.com>
 * 
 * First built on 2021.04.04
 */


#include "leader_estimator.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "leader_estimator");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  leaderEstimate* leaderEstimator = new leaderEstimate(nh, nh_private); 
  //create an object named leaderEstimator in heap rather than stack

  ros::spin();
  return 0;
}
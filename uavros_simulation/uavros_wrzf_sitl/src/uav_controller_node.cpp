/**
 * @brief UAV controller node
 *
 * Flight controller to output velocity setpoint
 *
 * @author Peixuan Shu <shupeixuan@qq.com>
 * 
 * First built on 2021.07.10
 */


#include "uav_controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "uav_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  uavCtrl* uavController = new uavCtrl(nh, nh_private); 
  //create an object named uavController in heap rather than stack

  ros::spin();
  return 0;
}
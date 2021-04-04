/**
 * @brief UAV controller node
 *
 * UAV controller to output acceleration setpoint
 *
 * @author Peixuan Shu <shupeixuan@qq.com>
 * 
 * First built on 2021.04.02
 */


#include "uavros_uavugv_sitl/uav_controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "uav_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  uavCtrl* uavController = new uavCtrl(nh, nh_private); 
  //create an object named uavController in heap rather than stack

  ros::spin();
  return 0;
}
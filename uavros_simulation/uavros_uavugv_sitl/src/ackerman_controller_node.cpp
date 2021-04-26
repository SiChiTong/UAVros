/**
 * @brief ackerman controller node
 *
 * ackerman controller to output setpoint
 *
 * @author Peixuan Shu <shupeixuan@qq.com>
 * 
 * First built on 2021.04.02
 */


#include "ackerman_controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ackerman_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ackermanCtrl* ackermanController = new ackermanCtrl(nh, nh_private); 
  //create an object named ackermanController in heap rather than stack

  ros::spin();
  return 0;
}
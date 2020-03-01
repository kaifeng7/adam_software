/**
 * @file can_driver_node.cpp
 * @author heng zhang (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2018-12-20
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#include "can_driver/can_driver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "can_driver_node");

  ros::NodeHandle nh, pnh("~");
  CanDriver can(nh, pnh);
  can.run();

  return 0;
}
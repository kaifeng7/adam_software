/**
 * @file ethernet_viewer_node.cpp
 * @author heng zhang (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2018-11-26
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#include "ethernet_viewer/ethernet_viewer.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ethernet_viewer_node");
  ros::NodeHandle nh, pnh("~");
  EthernetViewer viewer(nh, pnh);
  //viewer.run();
  //ros::spin();
  return 0;
}
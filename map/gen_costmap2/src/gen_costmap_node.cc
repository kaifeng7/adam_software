/**
 * @file gen_costmap_node.cc
 * @author heng zhang (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2019-01-22
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <ros/ros.h>
#include "update_costmap.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "gen_costmap2");

  ros::NodeHandle nh, pnh("~");
  UpdateCostmap uc(nh, pnh);
  if (!uc.init()){
    ROS_ERROR("failed to init UpdateCostmap.");
    return -1;
  }
  uc.run();

  return 0;
}
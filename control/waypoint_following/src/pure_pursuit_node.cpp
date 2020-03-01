/*
 * @Author: fengk 
 * @Date: 2019-04-12 14:56:07 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-16 11:37:18
 */


#include "pure_pursuit.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_following");
  PurePursuit pure_pursuit;
  pure_pursuit.MainLoop();

  return 0;
}

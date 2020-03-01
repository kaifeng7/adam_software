/*
 * @Author: fengk 
 * @Date: 2019-04-03 17:34:12 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-16 10:29:48
 */
#include "load_vectormap.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "vectormap_loading");

  LoadMap loadmap;
  loadmap.MainLoop();

  return 0;
}

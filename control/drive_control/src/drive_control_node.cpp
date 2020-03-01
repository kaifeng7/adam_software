#include <ros/ros.h>
#include "drive_control/drive_control.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_control_node");
  ros::NodeHandle nh, pnh("~");
  DriveControl dv(nh, pnh);
  dv.run();
  return 0;
}
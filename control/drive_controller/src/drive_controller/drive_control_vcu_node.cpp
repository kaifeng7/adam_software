#include <ros/ros.h>
#include "drive_controller/drive_control_vcu.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_control_node");
  ros::NodeHandle nh, pnh("~");
  DriveControl dv(nh, pnh);

  ros::spin();

  return 0;
}

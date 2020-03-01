/*
 * @Author: fengk 
 * @Date: 2019-04-02 16:00:51 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-02 20:41:45
 */
#include <ros/ros.h>
#include <adam_msgs/CarBasicInfo.h>

class CarBasicInfo
{
  public:
    adam_msgs::CarBasicInfo car_basic_info;
    ros::Publisher pub_CarBasicInfo;

    CarBasicInfo();
    ~CarBasicInfo();
    void MainLoop();

};

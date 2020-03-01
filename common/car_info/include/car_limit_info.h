/*
 * @Author: fengk 
 * @Date: 2019-04-02 17:47:23 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-02 20:41:42
 */
#include <ros/ros.h>
#include <adam_msgs/CarLimitInfo.h>

class CarLimitInfo
{
  public:
    adam_msgs::CarLimitInfo car_limit_info;
    ros::Publisher pub_CarLimitInfo;

    CarLimitInfo();
    ~CarLimitInfo();
    void MainLoop();

};

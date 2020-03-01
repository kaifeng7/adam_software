/*
 * @Author: fengk 
 * @Date: 2019-04-02 16:07:42 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-02 17:44:50
 */
#include "car_basic_info.h"

CarBasicInfo::CarBasicInfo()
{
    ros::NodeHandle nh_, pnh_("~");

    pnh_.param<double>("width", car_basic_info.width, 0.5);
    pnh_.param<double>("length", car_basic_info.length, 0.6);
    pnh_.param<double>("wheelBaseLength", car_basic_info.wheel_base, 0.4);
    pnh_.param<double>("turningRadius", car_basic_info.turning_radius, 5.2);

    pub_CarBasicInfo = nh_.advertise<adam_msgs::CarBasicInfo>("car_basic_info", 1);
}
CarBasicInfo::~CarBasicInfo() {}

void CarBasicInfo::MainLoop()
{
    ROS_INFO_STREAM("car_basic_info node start");
    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        ros::spinOnce();
        pub_CarBasicInfo.publish(car_basic_info);

        loop_rate.sleep();
    }
}
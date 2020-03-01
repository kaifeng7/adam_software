/*
 * @Author: fengk 
 * @Date: 2019-04-02 19:38:19 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-02 19:44:08
 */
#include "car_limit_info.h"

CarLimitInfo::CarLimitInfo()
{
    ros::NodeHandle nh_, pnh_("~");

    pnh_.param<double>("max_acceleration", car_limit_info.max_acceleration, 0.5);
    pnh_.param<double>("max_deceleration", car_limit_info.max_deceleration, 0.6);
    pnh_.param<double>("max_brake_value", car_limit_info.max_brake_value, 0.4);
    pnh_.param<double>("min_brake_value", car_limit_info.min_brake_value, 0.4);
    pnh_.param<double>("max_speed_backword", car_limit_info.max_speed_backword, 0.4);
    pnh_.param<double>("max_speed_forward", car_limit_info.max_speed_forward, 0.4);
    pnh_.param<double>("min_speed_forward", car_limit_info.min_speed_forward, 0.4);
    pnh_.param<double>("max_steer_angle", car_limit_info.max_steer_angle, 0.4);
    pnh_.param<double>("min_steer_angle", car_limit_info.min_steer_angle, 0.4);
    pnh_.param<double>("max_steer_value", car_limit_info.max_steer_value, 0.4);
    pnh_.param<double>("min_steer_value", car_limit_info.min_steer_value, 0.4);

    pub_CarLimitInfo = nh_.advertise<adam_msgs::CarLimitInfo>("car_limit_info", 1);
}
CarLimitInfo::~CarLimitInfo() {}

void CarLimitInfo::MainLoop()
{
    ROS_INFO_STREAM("car_Limit_info node start");
    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        ros::spinOnce();
        pub_CarLimitInfo.publish(car_limit_info);

        loop_rate.sleep();
    }
}
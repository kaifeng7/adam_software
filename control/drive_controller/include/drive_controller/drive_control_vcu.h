#ifndef DRIVE_CONTROLLER_DRIVECONTROL_H
#define DRIVE_CONTROLLER_DRIVECONTROL_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <adam_msgs/VehicleCmd.h>

#include "drive_controller/can/can.h"


class DriveControl{
    public:
    DriveControl(ros::NodeHandle nh, ros::NodeHandle pnh);

    private:

    void odomCB(const nav_msgs::Odometry::ConstPtr &msg);
    void cmdCB(const adam_msgs::VehicleCmd::ConstPtr &msg);


    Can can2VCU_;
    Can canFromVCU_;

    nav_msgs::Odometry cur_odom_;
    adam_msgs::VehicleCmd cur_cmd_;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber sub_cmd_;
    ros::Subscriber sub_odom_;

    std::string can_name_;

    // 车辆参数
    float max_speed_f_;  // 前进最大速度
    float max_speed_b_;  // 后退最大速度

    float max_acc_time_; // 最大加速时间
    float min_acc_time_; // 最小加速时间
    float defalut_acc_time_;

    float max_cmd_f_;  // 前进最大速度对应的指令值
    float max_cmd_b_;  // 后退最大速度对应的指令值

    float max_angle_;
    float min_angle_;

    float max_angle_cmd_;
    float min_angle_cmd_;

    float angle_resolution_;
    float steering_wheel_ratio_;

    float default_angle_acc_;   // 转角加速度
    float last_target_angle_;   // 上次目标转角
    ros::Time last_cmd_send_time_;

    int cur_mode_ = 1;
    bool first_cmd_flag_ = true;


};

#endif // DRIVE_CONTROLLER_DRIVECONTROL_H

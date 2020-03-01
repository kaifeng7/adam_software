#include "drive_controller/drive_control_vcu.h"
 

DriveControl::DriveControl(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
{
    sub_cmd_ = nh_.subscribe<adam_msgs::VehicleCmd>("/vehicle_cmd", 1, boost::bind(&DriveControl::cmdCB, this, _1));

    pnh_.param<std::string>("can_interface", can_name_, "can1");
    pnh_.param<float>("max_speed_f", max_speed_f_, 3);
    pnh_.param<float>("max_speed_b", max_speed_b_, 1);
    pnh_.param<float>("max_acc_time", max_acc_time_, 25.5);
    pnh_.param<float>("min_acc_time", min_acc_time_, 0.2);
    pnh_.param<float>("default_acc_time", defalut_acc_time_, 1.0);
    pnh_.param<float>("max_cmd_f", max_cmd_f_, 1162);
    pnh_.param<float>("max_cmd_b", max_cmd_b_, 500);
    pnh_.param<float>("max_angle", max_angle_, M_PI / 4);
    pnh_.param<float>("min_angle", min_angle_, -M_PI / 4);
    pnh_.param<float>("max_angle_cmd", max_angle_cmd_, 1220);
    pnh_.param<float>("min_angle_cmd", min_angle_cmd_, 580);
    pnh_.param<float>("default_angle_acc", default_angle_acc_, M_PI / 6);
    pnh_.param<float>("angle_resolution", angle_resolution_, 0.5);
    pnh_.param<float>("steering_wheel_ratio", steering_wheel_ratio_, 1.5);

    can2VCU_.init(can_name_, 0x203);

    Can cantest(can_name_, 0x103);
    uint8_t data[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    cantest.sendStandard(data, sizeof(data));
}

void DriveControl::odomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
    cur_odom_ = *msg;
}

void DriveControl::cmdCB(const adam_msgs::VehicleCmd::ConstPtr &msg)
{
    if(first_cmd_flag_){
        last_cmd_send_time_ = ros::Time::now();
        first_cmd_flag_ = false;
        last_target_angle_ = 0;
    }
    cur_cmd_ = *msg;

    uint8_t cmd2VCU[8];

    cmd2VCU[0] = 0x09; // brake
    cmd2VCU[0] = msg->mode == 1 ? 0x03 : msg->mode == 0 ? 0x05 : 0x09;

    float max_speed = 0;
    float max_cmd = 0;

    float target_speed = 0;

    if (msg->mode == 1)
    {
        if(msg->ctrl_cmd.speed < 0){
            // 模式错误
            target_speed = 0;
        }
        else {
            max_speed = max_speed_f_;
            max_cmd = max_cmd_f_;
            target_speed = msg->ctrl_cmd.speed > max_speed ? max_speed : msg->ctrl_cmd.speed < 0 ? 0 : msg->ctrl_cmd.speed;
        }
    }
    else if(msg->mode == 0)
    {
        if(msg->ctrl_cmd.speed > 0){
            // 模式错误
            target_speed = 0;
        }
        else{
            max_speed = max_speed_b_;
            max_cmd = max_cmd_b_;
            float ctrl_cmd_speed = -msg->ctrl_cmd.speed;
            target_speed = ctrl_cmd_speed > max_speed
                               ? max_speed
                               : ctrl_cmd_speed < 0 ? 0 : ctrl_cmd_speed;
        }
        
    }
    else{
        target_speed = 0;
    }


    uint16_t speed_set = (uint16_t)(target_speed / max_speed * max_cmd);

    cmd2VCU[1] = speed_set & 0x00ff;
    cmd2VCU[2] = speed_set >> 8;
    ROS_WARN("target_speed: %f, speed_set:%u", target_speed, speed_set);

    float acc_time = msg->ctrl_cmd.acceleration > max_acc_time_ ? max_acc_time_ : msg->ctrl_cmd.acceleration < min_acc_time_ ? min_acc_time_ : msg->ctrl_cmd.acceleration;
    // 无acc指令时
    if(std::abs(msg->ctrl_cmd.acceleration) < 0.001){
        acc_time = defalut_acc_time_;
    }

    uint8_t acc_set = (uint8_t)(acc_time * 10);
    cmd2VCU[3] = acc_set;

    float target_angle = msg->ctrl_cmd.steering_angle > max_angle_ ? max_angle_ : msg->ctrl_cmd.steering_angle < min_angle_ ? min_angle_ : msg->ctrl_cmd.steering_angle;
    printf("msg->steerign=%f\n", msg->ctrl_cmd.steering_angle);
    printf("target_angle1=%f\n", target_angle);

    ros::Time timeNow = ros::Time::now();
    
    float max_angle_delta = default_angle_acc_ * (timeNow - last_cmd_send_time_).toSec();
    
    printf("duration sec = %f, nsec = %f\n", (timeNow - last_cmd_send_time_).toSec(), 0);
    printf("delta_max=%f\n", max_angle_delta);
    if(target_angle - last_target_angle_ >= 0){

        target_angle = (target_angle - last_target_angle_) > max_angle_delta ? (last_target_angle_ + max_angle_delta) : target_angle;
    }
    else{
        target_angle = (target_angle - last_target_angle_) < -max_angle_delta ? (last_target_angle_ - max_angle_delta) : target_angle;
    }
    printf("target_angle2=%f\n" ,target_angle);

    float eps_target_angle = target_angle * steering_wheel_ratio_;
    
    int16_t angle_set;
    float zero_angle_cmd = (max_angle_cmd_ + min_angle_cmd_) * 0.5;

    angle_set = eps_target_angle / M_PI * 180 / angle_resolution_;
    angle_set = angle_set > max_angle_cmd_ ? max_angle_cmd_ : angle_set < min_angle_cmd_ ? min_angle_cmd_ : angle_set;

    // if (target_angle > 0)
    // {
    //     angle_set = target_angle / max_angle_ * (max_angle_cmd_ - zero_angle_cmd) + zero_angle_cmd;
    // }
    // else
    // {
    //     angle_set = target_angle / min_angle_ * (min_angle_cmd_ - zero_angle_cmd) + zero_angle_cmd;
    // }
    ROS_WARN("target_angle: %f, angle_set %d", target_angle, angle_set);

    cmd2VCU[5] = angle_set & 0x00ff;
    cmd2VCU[6] = angle_set >> 8;

    if (msg->mode != cur_mode_ && cur_mode_ != 2)
    {
        cmd2VCU[0] = 0x09;
        // 速度为零时再切换, todo
        ros::Duration(2).sleep();
        can2VCU_.sendStandard(cmd2VCU,sizeof(cmd2VCU));
    }
    else
    {
        can2VCU_.sendStandard(cmd2VCU, sizeof(cmd2VCU));
    }

    cur_mode_ = msg->mode;

    last_target_angle_ = target_angle;
    last_cmd_send_time_ = timeNow;
}

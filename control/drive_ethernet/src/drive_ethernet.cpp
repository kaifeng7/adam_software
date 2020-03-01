#include "drive_ethernet.h"

bool DriveEthernet::run()
{
    ros::spin();
    return 0;

}

bool DriveEthernet::init()
{
    ROS_INFO("Drive ethernet start init");

    if(!send.connect_stm32("192.168.1.101",10008))
    {
        ROS_WARN("Failed to connect:");
        return -1;
    }

    sub_cmd = nh_.subscribe<adam_msgs::VehicleCmd>("/vehicle_cmd",1,boost::bind(&DriveEthernet::cmd_callback,this,_1));
}


void DriveEthernet::cmd_callback(const adam_msgs::VehicleCmd::ConstPtr &msg)
{
    int mode = msg->mode;
    double speed = msg->ctrl_cmd.speed;
    double angle = msg->ctrl_cmd.steering_angle;
    send.send_stm32(mode,speed,angle);
    //send.send_stm32(1,1,1);
}


#include <ros/ros.h>
#include <string>
#include <adam_msgs/pid.h>
#include "drive_controller/can/can.h"

Can can204;

void pidCB(const adam_msgs::pid::ConstPtr &msg){
    double p = msg->Kp;
    double i = msg->Ki;
    double d = msg->Kd;

    uint16_t pint = (uint16_t)(p * 10000);
    uint16_t iint = (uint16_t)(i * 10000);
    uint16_t dint = (uint16_t)(d * 10000);

    uint8_t send204[8] = {0};
    
    send204[0] = (pint >> 8) & 0xff;
    send204[1] = pint & 0xff;
    send204[2] = (iint >> 8) & 0xff;
    send204[3] = iint & 0xff;
    send204[4] = (dint >> 8) & 0xff;
    send204[5] = dint & 0xff;
    
    can204.sendStandard(send204, sizeof(send204));
    ROS_INFO("send %d, %d, %d",pint, iint ,dint );
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "send_pid_node");
    ros::NodeHandle nh, pnh("~");

    std::string can_interface;
    pnh.param<std::string>("can_interface", can_interface, "can1");
    can204.init(can_interface, 0x204);

    ros::Subscriber pid_sub = nh.subscribe<adam_msgs::pid>("/pid_params", 1, boost::bind(&pidCB, _1));



    ros::spin();

    return 0;
}

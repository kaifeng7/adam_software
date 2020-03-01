#include <ros/ros.h>
#include <adam_msgs/VCUEPS.h>
#include "drive_controller/can/can.h"
#include <string>

int main(int argc, char **argv) {
    ros::init(argc, argv, "vcu_eps_node");
    ros::NodeHandle nh, pnh("~");

    ros::Publisher vcu_pub = nh.advertise<adam_msgs::VCUEPS>("/vcu_eps", 5);

    std::string can_interface;
    pnh.param<std::string>("can_interface", can_interface, "can1");
    Can can334(can_interface, 0x334);

    uint8_t data333[8];

    adam_msgs::VCUEPS eps_msgs;

    while (ros::ok()) {
        can334.receive(data333);
        eps_msgs.p = (data333[0] + (data333[1] << 8)) * 0.0001;
        eps_msgs.i = (data333[2] + (data333[3] << 8)) * 0.0001;
        eps_msgs.d = (data333[4] + (data333[5] << 8)) * 0.0001;
        eps_msgs.eps_duty = (data333[6] + (data333[7] << 8));

        vcu_pub.publish(eps_msgs);
    }

    return 0;
}

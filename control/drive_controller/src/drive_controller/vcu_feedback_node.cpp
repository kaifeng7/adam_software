#include <ros/ros.h>
#include "drive_controller/can/can.h"
#include "adam_msgs/VCU.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vcu_feedback_node");
    ros::NodeHandle nh, pnh("~");

    ros::Publisher vcu_pub= nh.advertise<adam_msgs::VCU>("/vcu_feedback", 5);

    std::string can_interface;
    pnh.param<std::string>("can_interface", can_interface, "can1");
    Can can183(can_interface, 0x183);
    Can can283(can_interface, 0x283);

    uint8_t data183[8];
    uint8_t data283[8];

    adam_msgs::VCU vcu_msgs;

    while(ros::ok()){
        can183.receive(data183);
        can283.receive(data283);
        int msH,msL,aH,aL;
        msL = data183[0];
        msH = data183[1];
        aL = data183[2];
        aH = data183[3];
        
        vcu_msgs.motorSpeed = data183[0] + (data183[1] << 8);
        vcu_msgs.angle = data183[2] + (data183[3] << 8);
        vcu_msgs.EPSFaultFlag = (data183[4] & 0x01);
        vcu_msgs.controllerFaultFlag = (data183[4] & 0x02);
        vcu_msgs.batteryTem = data183[5];
        vcu_msgs.brakeFlag = (data183[6] & 0x01);
        vcu_msgs.autoDriveFlag = (data183[6] & 0x02);
        vcu_msgs.motorCurrent = data283[0] + (data283[1] << 8);
        vcu_msgs.speed = data283[2];

        vcu_pub.publish(vcu_msgs);
    }

    return 0;
}

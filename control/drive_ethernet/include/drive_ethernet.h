#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <adam_msgs/ControlCommandStamped.h>
#include <adam_msgs/VehicleCmd.h>
#include "stm32_client.h"

class DriveEthernet
{
    public:
        DriveEthernet(ros::NodeHandle nh,ros::NodeHandle pnh):nh_(nh),pnh_(pnh){
            init();

        }
        ~DriveEthernet(){};
        bool run();
        bool init();
        stm32client send;
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_cmd;
        adam_msgs::ControlCommandStamped control_msgs;
        void cmd_callback(const adam_msgs::VehicleCmd::ConstPtr &msg);
};
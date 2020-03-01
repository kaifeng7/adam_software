#include "drive_ethernet.h"
int main(int argc,char **argv)
{
    ros::init(argc,argv,"drive_ethernet");
    ros::NodeHandle nh,pnh("~");
    DriveEthernet de(nh,pnh);
    de.run();
    return 0;
}
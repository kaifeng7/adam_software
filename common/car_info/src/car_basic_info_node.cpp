/*
 * @Author: fengk 
 * @Date: 2019-04-02 16:21:22 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-02 17:28:20
 */
#include "car_basic_info.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"car_basic_info_node");
    CarBasicInfo car_basic_info;
    car_basic_info.MainLoop();
    return 0;
}
/*
 * @Author: fengk 
 * @Date: 2019-04-02 19:40:10 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-02 19:40:42
 */
#include "car_limit_info.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"car_limit_info_node");
    CarLimitInfo car_limit_info;
    car_limit_info.MainLoop();
    return 0;
}
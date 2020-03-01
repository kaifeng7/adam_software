/*
 * @Author: fengk 
 * @Date: 2019-04-03 19:48:14 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-16 10:31:00
 */
#include "global_planning.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planning");
    GlobalPlanning global_plan;
    global_plan.MainLoop();
    return 0;
}

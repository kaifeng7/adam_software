/*
 * @Author: fengk 
 * @Date: 2019-04-02 20:29:26 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-09 22:40:22
 */

#include "local_planning.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "local_planning");
	TrajectoryGen trajectory_gen;
	trajectory_gen.MainLoop();
	return 0;
}

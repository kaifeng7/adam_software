/*
 * @Author: fengk 
 * @Date: 2019-04-17 17:44:16 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-17 17:47:43
 */
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "utility.h"

nav_msgs::Path path;
ros::Publisher pub_;
bool bPath;
Utility utility;

void pathCallBack(const nav_msgs::PathConstPtr msg)
{
    path = *msg;
    bPath = true;
    //TODO 
    //path_hz should < pose_hz
}
void poseCallBack(const geometry_msgs::PoseStampedConstPtr &msg)
{
    if (!bPath)
    {
        ROS_WARN("No path subscriber");
        return;
    }

    double min = DBL_MAX;
    int min_i = -1;
    nav_msgs::Path pub_msg;

    for (int i = 0; i < utility.getSize(path); i++)
    {
        double distance = utility.getPlaneDistance(path.poses.at(i).pose.position, msg->pose.position);
        if (distance < min)
        {
            min = distance;
            min_i = i;
        }
    }
    ROS_INFO("update path");
    pub_msg.header = path.header;
    for (;min_i < (int)path.poses.size();min_i++)
    {
        pub_msg.poses.push_back(path.poses.at(min_i));
    }
    ROS_INFO_STREAM("final paths = "<<pub_msg.poses.size());
    pub_.publish(pub_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_path");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe("sim_pose", 100, poseCallBack);
    ros::Subscriber path_sub = nh.subscribe("/loaded_trajectory/recorded_path", 100, pathCallBack);
    pub_ = nh.advertise<nav_msgs::Path>("final_waypoints", 100);
    ros::spin();
    return 0;
}
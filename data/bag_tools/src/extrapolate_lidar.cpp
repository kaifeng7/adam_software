/**
 * @file extrapolate_lidar.cpp
 * @author heng zhang (you@domain.com)
 * @brief 通过 scan_id 降采样激光雷达。如：从 16 线降采样到 8 线
 * @version 0.1
 * @date 2019-04-9
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <ros/duration.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <string>
#include <vector>
#include <unordered_set>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

vector<int> param_scan_id_;
vector<string> param_topics_;
string param_bag_name_i_;
string param_bag_name_o_;

unordered_set<int> scan_ids_;

rosbag::Bag bag_i_;
rosbag::Bag bag_o_;
rosbag::View view_i_;
ros::Publisher pub_pc_;

bool init(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
{
  if (!pnh.getParam("scan_id", param_scan_id_))
  {
    ROS_ERROR("cannot get scan_id");
    return false;
  }
  else
  {
    for (const auto &id : param_scan_id_)
    {
      scan_ids_.insert(id);
    }
  }
  if (!pnh.getParam("topics", param_topics_))
  {
    ROS_ERROR("cannot get topics");
    return false;
  }
  pnh.param<string>("bag_name_i", param_bag_name_i_, "");
  pnh.param<string>("bag_name_o", param_bag_name_o_, "");

  try
  {
    ROS_INFO("openning input bag: %s", param_bag_name_i_.c_str());
    bag_i_.open(param_bag_name_i_, rosbag::bagmode::Read);
    ROS_INFO("openning output bag: %s", param_bag_name_o_.c_str());
    bag_o_.open(param_bag_name_o_, rosbag::bagmode::Write);
  }
  catch (ros::Exception &e)
  {
    ROS_ERROR("cannot open bag");
    return false;
  }

  view_i_.addQuery(bag_i_, rosbag::TopicQuery(param_topics_));
  ROS_INFO("param_topics length %d", param_topics_.size());

  return true;
}

void extrapolate(rosbag::View &view, rosbag::Bag &bag_o)
{
  BOOST_FOREACH (rosbag::MessageInstance const m, view)
  {
    if (m.getTopic() == "/lslidar_point_cloud")
    {
      const sensor_msgs::PointCloud2::ConstPtr &msg_pc = m.instantiate<sensor_msgs::PointCloud2>();
      // ROS_INFO("frame: %s, stamp: %f, points: %d", (*msg_pc).header.frame_id.c_str(), msg_pc->header.stamp.toSec(), msg_pc->width * msg_pc->height);
      // pub_pc_.publish(*msg_pc);
      // ros::Duration duration(0.1);
      // duration.sleep();
      pcl::PointCloud<pcl::PointXYZ> pcl_pc;
      pcl::fromROSMsg(*msg_pc, pcl_pc);
      // ROS_INFO("test");
      pcl::PointCloud<pcl::PointXYZ> pcl_pc_out;

      double angle;
      int id;
      for (int i = 0; i < pcl_pc.size(); ++i)
      {
        const pcl::PointXYZ &p = pcl_pc[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        {
          continue;
        }
        angle = -std::atan2(p.z, std::hypot(p.x, p.y));
        id = int((angle * 180 / M_PI + 15) / 2 + 0.5);
        if (id < 0 || id > 15)
        {
          ROS_WARN("Error in computing scan_id: %d", id);
        }
        else
        {
          if (scan_ids_.count(id))
          {
            pcl_pc_out.points.push_back(p);
          }
        }
      }
      sensor_msgs::PointCloud2 msg_pc_out;
      pcl::toROSMsg(pcl_pc_out, msg_pc_out);
      msg_pc_out.header = msg_pc->header;
      bag_o.write(m.getTopic(), m.getTime(), msg_pc_out);
    }
    else if (m.getTopic() == "/tf")
    {
      tf2_msgs::TFMessage::ConstPtr tmp_msg = m.instantiate<tf2_msgs::TFMessage>();
      bag_o.write(m.getTopic(), m.getTime(), *tmp_msg);
    }
    else if (m.getTopic() == "/imu/data")
    {
      sensor_msgs::Imu::ConstPtr tmp_msg = m.instantiate<sensor_msgs::Imu>();
      bag_o.write(m.getTopic(), m.getTime(), *tmp_msg);
    }
    else if (m.getTopic() == "/odom/imu")
    {
      nav_msgs::Odometry::ConstPtr tmp_msg = m.instantiate<nav_msgs::Odometry>();
      bag_o.write(m.getTopic(), m.getTime(), *tmp_msg);
    }
    else if (m.getTopic() == "/wheel_circles")
    {
      geometry_msgs::TwistStamped::ConstPtr tmp_msg = m.instantiate<geometry_msgs::TwistStamped>();
      bag_o.write(m.getTopic(), m.getTime(), *tmp_msg);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "extrapolate_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  if (!init(nh, pnh))
  {
    ROS_ERROR("error in init, exit...");
    exit(-1);
  }

  pub_pc_ = nh.advertise<sensor_msgs::PointCloud2>("/test", 1);

  ROS_INFO("extrapolate start.");

  extrapolate(view_i_, bag_o_);
  bag_o_.close();

  ROS_INFO("extrapolate end.");

  return 0;
}
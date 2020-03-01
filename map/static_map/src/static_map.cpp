/**
 * @file static_map.cpp
 * @author heng zhang (you@domain.com)
 * @brief 载入 pcd 文件发送点云话题，一般用来载入地图，可以通过设置 use_local_map 来加载部分地图
 * @version 0.1
 * @date 2018-12-28
 * 
 * @copyright Copyright (c) 2018
 * 
 */
#include <ros/ros.h>
#include <ros/duration.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pthread.h>

bool param_use_local_map_;  // 是否使用局部地图，当地图太大时为了能在显存受限的条件下正常运行定位应当启用此参数
double param_local_map_wl_; // 局部地图宽
double param_local_map_wr_;
double param_local_map_hf_; // 局部地图高
double param_local_map_hr_;
double param_local_margin_wl_; // 定位在 margin 中时更新局部地图
double param_local_margin_wr_;
double param_local_margin_hf_;
double param_local_margin_hr_;
geometry_msgs::Pose map_origin_;
geometry_msgs::Point map_tr_; // top-right
geometry_msgs::Point map_dl_; // down-left
geometry_msgs::Point margin_tr_;
geometry_msgs::Point margin_dl_;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
sensor_msgs::PointCloud2 *msg_map_;
sensor_msgs::PointCloud2 *msg_local_map_;
PointCloudT *pcl_map_;
PointCloudT *pcl_local_map_;
tf::Transform tf_m2p_;
bool local_map_init_;
std::vector<PointT> test_v;

pthread_mutex_t mutex_;

/**
 * @brief 提取 center 附近的点云，没有考虑 center 的朝向，后续可以考虑增加，虽然点云是 360 度，但方向一定程度上反映了接下来的运动方向
 * 
 * @param center 
 * @param from 
 * @param to 
 * @return true 
 * @return false 
 */
bool substractMap(const geometry_msgs::Pose &center, PointCloudT *from, PointCloudT *to)
{
  pthread_mutex_lock(&mutex_);
  tf_m2p_.setOrigin(tf::Vector3(center.position.x, center.position.y, center.position.z));
  tf_m2p_.setRotation(tf::Quaternion(center.orientation.x, center.orientation.y, center.orientation.z, center.orientation.w));
  map_origin_ = center;
  map_tr_.x = center.position.x + param_local_map_hf_;
  map_tr_.y = center.position.y + param_local_map_wr_;
  map_dl_.x = center.position.x - param_local_map_hr_;
  map_dl_.y = center.position.y - param_local_map_wl_;
  to->points.clear();
  // to->points.resize(50000);
  test_v.push_back(PointT());
  double r = param_local_map_hf_ * param_local_map_hf_;
  for (auto p : pcl_map_->points)
  {
    // if (p.x < map_tr_.x && p.x > map_dl_.x && p.y < map_tr_.y && p.y > map_dl_.y)
    // {
    //   to->points.push_back(p);
    // }
    if ((std::pow(p.x - center.position.x, 2) + std::pow(p.y - center.position.y, 2)) < r)
    {
      to->points.push_back(p);
    }
  }
  pthread_mutex_unlock(&mutex_);
  ROS_INFO("new local map set: %d points. origin: x %f, y %f", to->points.size(), center.position.x, center.position.y);
  return true;
}

void poseCB(const geometry_msgs::PoseStampedConstPtr &msg)
{
  if ((std::pow(msg->pose.position.x - map_origin_.position.x, 2) + std::pow(msg->pose.position.y - map_origin_.position.y, 2)) > param_local_margin_wl_)
  {
    local_map_init_ = substractMap(msg->pose, pcl_map_, pcl_local_map_);
    pcl::toROSMsg(*pcl_local_map_, *msg_local_map_);
  }
}

void initialPoseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  local_map_init_ = substractMap(msg->pose.pose, pcl_map_, pcl_local_map_);
  ROS_INFO("test: %d", pcl_local_map_->size());
  pcl::toROSMsg(*pcl_local_map_, *msg_local_map_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "static_map_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Publisher pub_map;
  ros::Duration duration;

  std::string param_pcd_file;
  std::string param_map_frame;
  double param_duration;

  pnh.param<std::string>("pcd_file", param_pcd_file, "");
  pnh.param<std::string>("map_frame", param_map_frame, "/map");
  pnh.param<double>("duraion", param_duration, 1.0);
  pnh.param<bool>("use_local_map", param_use_local_map_, false);
  pnh.param<double>("local_map_wl", param_local_map_wl_, 5.);
  pnh.param<double>("local_map_wr", param_local_map_wr_, 5.);
  pnh.param<double>("local_map_hf", param_local_map_hf_, 15.);
  pnh.param<double>("local_map_hr", param_local_map_hr_, 5.);
  pnh.param<double>("local_margin_wl", param_local_margin_wl_, 2.5);
  pnh.param<double>("local_margin_wr", param_local_margin_wr_, 2.5);
  pnh.param<double>("local_margin_hf", param_local_margin_hf_, 7.5);
  pnh.param<double>("local_margin_hr", param_local_margin_hr_, 2.5);
  duration.fromSec(param_duration);
  pthread_mutex_init(&mutex_, NULL);

  msg_map_ = new sensor_msgs::PointCloud2();
  msg_local_map_ = new sensor_msgs::PointCloud2();
  pcl_map_ = new PointCloudT();
  pcl_local_map_ = new PointCloudT();
  if (param_pcd_file == "" || pcl::io::loadPCDFile(param_pcd_file, *msg_map_) == -1)
  {
    ROS_ERROR("Failed to load pcd file: %s", param_pcd_file.c_str());
    return (-1);
  }

  // pcl_local_map_->points = new std::vector<PointT>;
  pcl::fromROSMsg(*msg_map_, *pcl_map_);
  ROS_INFO("use_local_map: %d", param_use_local_map_);
  ros::Subscriber sub_initialpose;
  ros::Subscriber sub_pose;
  if (param_use_local_map_)
  {
    sub_initialpose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, initialPoseCB);
    sub_pose = nh.subscribe<geometry_msgs::PoseStamped>("/current_pose", 1, poseCB);
  }

  pub_map = nh.advertise<sensor_msgs::PointCloud2>("/static_map", 1);

  while (ros::ok())
  {
    if (param_use_local_map_ && local_map_init_)
    {
      msg_local_map_->header.stamp = ros::Time::now();
      msg_local_map_->header.frame_id = param_map_frame;
      pub_map.publish(*msg_local_map_);
    }
    else if (!param_use_local_map_)
    {
      msg_map_->header.stamp = ros::Time::now();
      msg_map_->header.frame_id = param_map_frame;
      pub_map.publish(*msg_map_);
    }

    ros::spinOnce();
    duration.sleep();
  }
  ros::spin();

  return 0;
}
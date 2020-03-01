/**
 * @file update_costmap.h
 * @author heng zhang (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2019-01-22
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef __GEN_COSTMAP2__
#define __GEN_COSTMAP2__

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <gen_costmap2/GenCostmap2Config.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <string>
#include <cmath>
#include <pthread.h>
#include <limits>
#include <vector>
#include <algorithm>

#include "utils.hpp"

class UpdateCostmap
{
public:
  inline UpdateCostmap(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh){};
  inline ~UpdateCostmap()
  {
  }
  bool init();
  void run();

private:
  typedef struct
  {
    float rad;
    float dist;
    int x;
    int y;
  } fake_point;
  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  std::string NAME_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_obstacle_pc_;
  ros::Subscriber sub_omap_;
  ros::Subscriber sub_pose_;
  nav_msgs::OccupancyGrid msg_map_;
  geometry_msgs::PoseStamped msg_pose_;
  tf::TransformListener tf_listener_;

  void poseCB(const geometry_msgs::PoseStampedConstPtr &msg);
  void obstacleCB(const sensor_msgs::PointCloud2ConstPtr &msg);
  void mapCB(const nav_msgs::OccupancyGridConstPtr &msg);
  void cfgCB(const gen_costmap2::GenCostmap2Config &config, uint32_t level);
  void getLine(const int &s_x, const int &s_y, const int &e_x, const int &e_y, std::vector<std::pair<int, int>> &points);

  ros::Publisher pub_updated_map_;
  ros::Publisher pub_viz_;
  nav_msgs::OccupancyGrid msg_updated_map_;

  // params
  std::string param_frame_map_;
  std::string param_frame_laser_;
  std::string param_frame_base_;
  double param_max_range_;        // 障碍物最远距离，默认 20m
  double param_angle_resolution_; // 角度分辨率，默认 0.2°，最好保证 param_max_range * param_angle_resolution / 180. * 3.14 ~ map.resolution，同时注意激光雷达自身的角度分辨率
  int param_decay_;               // cost 减小的步长

  tf::StampedTransform tf_b2l_;
  bool map_set_;
  bool pose_set_;
  int max_range_pixel_;               // 障碍物最远距离（对应栅格个数）：param_max_range / map.resolution
  int fake_num_rays_;                 // 将障碍物点云在栅格中的投影视为 fake 单线激光雷达的 scan，fake_num_rays ～ 360 / param_angle_resolution
  std::vector<fake_point> fake_scan_; // length = fake_num_rays 保存最近障碍物的距离
  std::vector<fake_point> fake_scan_t_;
  double fake_angle_resolution_; // 单位为 rad

  pthread_mutex_t mutex_;
};

#endif
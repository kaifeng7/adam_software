/**
 * @file update_costmap.cc
 * @author heng zhang (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2019-01-22
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "update_costmap.h"

bool UpdateCostmap::init()
{
  ros::Duration(1.0).sleep();
  pnh_.param<std::string>("frame_map", param_frame_map_, "map");
  pnh_.param<std::string>("frame_laser", param_frame_laser_, "/laser");
  pnh_.param<std::string>("frame_base", param_frame_base_, "/base_link");
  pnh_.param<double>("max_range", param_max_range_, 20.);
  pnh_.param<double>("angle_resolution", param_angle_resolution_, 0.2);
  pnh_.param<int>("decay", param_decay_, 10);
  try
  {
    tf_listener_.waitForTransform(param_frame_base_, param_frame_laser_, ros::Time(0), ros::Duration(10.));
    tf_listener_.lookupTransform(param_frame_base_, param_frame_laser_, ros::Time(0), tf_b2l_);
  }
  catch (const tf::TransformException &ex)
  {
    ROS_ERROR_NAMED(NAME_, ex.what());
    return false;
  }
  pthread_mutex_init(&mutex_, NULL);
  fake_angle_resolution_ = ANGLE2RAD(param_angle_resolution_);
  fake_num_rays_ = int(std::ceil(2 * M_PI / fake_angle_resolution_));
  fake_scan_t_.resize(fake_num_rays_);
  fake_scan_.resize(fake_num_rays_);
  for (int i = 0; i < fake_scan_t_.size(); ++i)
  {
    fake_scan_t_[i].rad = i * fake_angle_resolution_;
    fake_scan_t_[i].dist = param_max_range_;
    fake_scan_t_[i].x = param_max_range_ * std::cos(fake_scan_t_[i].rad);
    fake_scan_t_[i].y = param_max_range_ * std::sin(fake_scan_t_[i].rad);
  }
  fake_scan_.insert(fake_scan_.end(), fake_scan_t_.begin(), fake_scan_t_.end());
  ROS_INFO("fake_scan_t.size(): %d; fake_scan.size(): %d", fake_scan_t_.size(), fake_scan_.size());
  ROS_INFO("fake_num_rays: %d", fake_num_rays_);
  msg_updated_map_.header.frame_id = param_frame_map_;

  pub_updated_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/updated_map", 1);
  pub_viz_ = nh_.advertise<visualization_msgs::Marker>("/viz_scan", 1);
  dynamic_reconfigure::Server<gen_costmap2::GenCostmap2Config> cfg_server;
  dynamic_reconfigure::Server<gen_costmap2::GenCostmap2Config>::CallbackType cfg_cb = boost::bind(&UpdateCostmap::cfgCB, this, _1, _2);
  cfg_server.setCallback(cfg_cb);

  ROS_INFO_NAMED(NAME_, "init.");

  return true;
}

void UpdateCostmap::run()
{

  sub_obstacle_pc_ = nh_.subscribe<sensor_msgs::PointCloud2>("/obstacle_pc", 1, boost::bind(&UpdateCostmap::obstacleCB, this, _1));
  sub_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/ndt/current_pose", 1, boost::bind(&UpdateCostmap::poseCB, this, _1));
  sub_omap_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/global_costmap", 1, boost::bind(&UpdateCostmap::mapCB, this, _1));

  ros::spin();
}

/**
 * @brief 将障碍物点云投影到二维栅格中，仅考虑 param_max_range_ 圆形范围内的障碍物，
 * 地图分辨率与全局地图分辨率一致，暂不考虑全局地图的朝向问题
 * 
 * @param msg 
 */
void UpdateCostmap::obstacleCB(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  if (!pose_set_ || !map_set_)
  {
    ROS_WARN_NAMED(NAME_, "pose or map not set yet.");
    return;
  }
  ROS_INFO("test");
  pthread_mutex_lock(&mutex_);

  // 获取 laser 位置
  geometry_msgs::Pose pose_laser;
  tf::Pose tf_m2b;
  tf::poseMsgToTF(msg_pose_.pose, tf_m2b);
  tf::poseTFToMsg(tf_m2b * tf_b2l_, pose_laser);
  tf::Quaternion tmp_q;
  tf::quaternionMsgToTF(pose_laser.orientation, tmp_q);
  tf::Matrix3x3 rotation(tmp_q);
  Eigen::Matrix4f m_m2l = Eigen::Matrix4f::Identity();
  m_m2l(0, 3) = pose_laser.position.x;
  m_m2l(1, 3) = pose_laser.position.y;
  m_m2l(2, 3) = pose_laser.position.z;
  m_m2l(0, 0) = rotation.getRow(0).getX();
  m_m2l(0, 1) = rotation.getRow(0).getY();
  m_m2l(0, 2) = rotation.getRow(0).getZ();
  m_m2l(1, 0) = rotation.getRow(1).getX();
  m_m2l(1, 1) = rotation.getRow(1).getY();
  m_m2l(1, 2) = rotation.getRow(1).getZ();
  m_m2l(2, 0) = rotation.getRow(2).getX();
  m_m2l(2, 1) = rotation.getRow(2).getY();
  m_m2l(2, 2) = rotation.getRow(2).getZ();

  fake_scan_.clear();
  fake_scan_.insert(fake_scan_.end(), fake_scan_t_.begin(), fake_scan_t_.end());
  for (auto &s : fake_scan_)
  {
    s.x = int((s.x + pose_laser.position.x - msg_map_.info.origin.position.x) / msg_map_.info.resolution);
    s.x = s.x < 0 ? 0 : (s.x >= msg_map_.info.width ? msg_map_.info.width - 1 : s.x);
    s.y = int((s.y + pose_laser.position.y - msg_map_.info.origin.position.y) / msg_map_.info.resolution);
    s.y = s.y < 0 ? 0 : (s.y >= msg_map_.info.height ? msg_map_.info.height - 1 : s.y);
  }

  // transform pointcloud from laser frame to map frame
  PointCloudT::Ptr pc_l(new PointCloudT()); // laser 坐标系
  PointCloudT::Ptr pc_m(new PointCloudT()); // map 坐标系
  pcl::fromROSMsg(*msg, *pc_l);
  if (pc_l->points.size() == 0)
  {
    ROS_INFO_NAMED(NAME_, "no obstacle.");
    pthread_mutex_unlock(&mutex_);
    return;
  }
  pcl::transformPointCloud(*pc_l, *pc_m, m_m2l);

  int rad_index = 0;
  // 雷达在栅格中的位置
  int l_x = int((pose_laser.position.x - msg_map_.info.origin.position.x) / msg_map_.info.resolution);
  int l_y = int((pose_laser.position.y - msg_map_.info.origin.position.y) / msg_map_.info.resolution);
  if (!(l_x >= 0 && l_x < msg_map_.info.width && l_y >= 0 && l_y < msg_map_.info.height))
  {
    ROS_ERROR("laser is out of occupancy map.");
    pthread_mutex_unlock(&mutex_);
    return;
  }
  int p_x = -1, p_y = -1;
  double tmp_rad = 0.;
  double laser_yaw = tmp_q.getAxis().getZ(); // 激光雷达在地图中的 yaw 值
  double dist = -1.;
  for (int i = 0; i < pc_m->size(); ++i)
  {
    // 观测点在地图中的位置
    p_x = int((pc_m->points[i].x - msg_map_.info.origin.position.x) / msg_map_.info.resolution);
    p_y = int((pc_m->points[i].y - msg_map_.info.origin.position.y) / msg_map_.info.resolution);
    tmp_rad = std::atan2(pc_l->points[i].y, pc_l->points[i].x);
    rad_index = (fake_num_rays_ + int(tmp_rad / fake_angle_resolution_)) % fake_num_rays_; // 投影在 laser 中的角度索引
    dist = std::hypot(pc_l->points[i].x, pc_l->points[i].y);

    if (dist >= fake_scan_[rad_index].dist)
    {
      // 忽略最大观测范围外的点
      // 只考虑最近的障碍物，忽略被遮挡的障碍物
      continue;
    }

    if (!(p_x >= 0 && p_x < msg_map_.info.width && p_y >= 0 && p_y < msg_map_.info.height))
    {
      // 忽略地图外的点
      continue;
    }

    // 进行 ray-casting 更新地图，更新 (l_x, l_y) 到 (p_x, p_y) 之间的栅格
    fake_scan_[rad_index].dist = dist;
    fake_scan_[rad_index].x = p_x;
    fake_scan_[rad_index].y = p_y;
  }
  ROS_INFO("fake_scan.size(): %d", fake_scan_.size());
  visualization_msgs::Marker msg_marker;
  msg_marker.header.frame_id = param_frame_map_;
  msg_marker.header.stamp = ros::Time();
  msg_marker.id = 0;
  msg_marker.type = visualization_msgs::Marker::LINE_LIST;
  msg_marker.action = visualization_msgs::Marker::ADD;
  msg_marker.pose.orientation.w = 1.0;
  msg_marker.scale.x = 0.1;
  msg_marker.scale.y = 0.1;
  msg_marker.scale.z = 0.1;
  msg_marker.color.a = 1.0; // Don't forget to set the alpha!
  msg_marker.color.r = 0.0;
  msg_marker.color.g = 1.0;
  msg_marker.color.b = 0.0;
  for (int i = 0; i < fake_scan_.size(); ++i)
  {
    geometry_msgs::Point tmp_p;
    tmp_p.x = msg_map_.info.origin.position.x + l_x * msg_map_.info.resolution;
    tmp_p.y = msg_map_.info.origin.position.y + l_y * msg_map_.info.resolution;
    msg_marker.points.push_back(tmp_p);
    tmp_p.x = msg_map_.info.origin.position.x + fake_scan_[i].x * msg_map_.info.resolution;
    tmp_p.y = msg_map_.info.origin.position.y + fake_scan_[i].y * msg_map_.info.resolution;
    msg_marker.points.push_back(tmp_p);
    std::vector<std::pair<int, int>> points;
    getLine(l_x, l_y, fake_scan_[i].x, fake_scan_[i].y, points);
    for (auto &p : points)
    {
      if (msg_updated_map_.data[p.second * msg_map_.info.width + p.first] >= param_decay_)
      {
        msg_updated_map_.data[p.second * msg_map_.info.width + p.first] -= param_decay_;
      }
      else
      {
        msg_updated_map_.data[p.second * msg_map_.info.width + p.first] = 0;
      }
    }
    if (fake_scan_[i].dist < param_max_range_)
    {
      msg_updated_map_.data[points[points.size() - 1].second * msg_map_.info.width + points[points.size() - 1].first] = 100;
    }
  }
  pthread_mutex_unlock(&mutex_);
  msg_updated_map_.header.stamp = msg->header.stamp;
  pub_updated_map_.publish(msg_updated_map_);
  pub_viz_.publish(msg_marker);
}

void UpdateCostmap::getLine(const int &s_x, const int &s_y, const int &e_x, const int &e_y, std::vector<std::pair<int, int>> &points)
{
  int dx, dy, incr1, incr2, d, x, y, xend, yend, xdirflag, ydirflag;

  int cnt = 0;

  dx = abs(e_x - s_x);
  dy = abs(e_y - s_y);

  if (dy <= dx)
  {
    d = 2 * dy - dx;
    incr1 = 2 * dy;
    incr2 = 2 * (dy - dx);
    if (s_x > e_x)
    {
      x = e_x;
      y = e_y;
      ydirflag = (-1);
      xend = s_x;
    }
    else
    {
      x = s_x;
      y = s_y;
      ydirflag = 1;
      xend = e_x;
    }
    points.push_back(std::make_pair(x, y));
    if (((e_y - s_y) * ydirflag) > 0)
    {
      while (x < xend)
      {
        x++;
        if (d < 0)
        {
          d += incr1;
        }
        else
        {
          y++;
          d += incr2;
        }
        points.push_back(std::make_pair(x, y));
      }
    }
    else
    {
      while (x < xend)
      {
        x++;
        if (d < 0)
        {
          d += incr1;
        }
        else
        {
          y--;
          d += incr2;
        }
        points.push_back(std::make_pair(x, y));
      }
    }
  }
  else
  {
    d = 2 * dx - dy;
    incr1 = 2 * dx;
    incr2 = 2 * (dx - dy);
    if (s_y > e_y)
    {
      y = e_y;
      x = e_x;
      yend = s_y;
      xdirflag = (-1);
    }
    else
    {
      y = s_y;
      x = s_x;
      yend = e_y;
      xdirflag = 1;
    }
    points.push_back(std::make_pair(x, y));
    if (((e_x - s_x) * xdirflag) > 0)
    {
      while (y < yend)
      {
        y++;
        if (d < 0)
        {
          d += incr1;
        }
        else
        {
          x++;
          d += incr2;
        }
        points.push_back(std::make_pair(x, y));
      }
    }
    else
    {
      while (y < yend)
      {
        y++;
        if (d < 0)
        {
          d += incr1;
        }
        else
        {
          x--;
          d += incr2;
        }
        points.push_back(std::make_pair(x, y));
      }
    }
  }

  if (s_x != points[0].first || s_y != points[0].second)
  {
    std::reverse(points.begin(), points.end());
  }
}

void UpdateCostmap::poseCB(const geometry_msgs::PoseStampedConstPtr &msg)
{
  msg_pose_ = *msg;
  pose_set_ = true;
}

void UpdateCostmap::mapCB(const nav_msgs::OccupancyGridConstPtr &msg)
{
  // 判断地图是否相同 TODO: 以后再改判断条件
  if (msg->info.height != msg_map_.info.height || msg->info.width != msg_map_.info.height)
  {
    pthread_mutex_lock(&mutex_);
    msg_map_ = *msg;
    msg_updated_map_.data.clear();
    msg_updated_map_.data.insert(msg_updated_map_.data.end(), msg->data.begin(), msg->data.end());
    msg_updated_map_.info = msg->info;
    // TODO: build lookup table
    pthread_mutex_unlock(&mutex_);
    map_set_ = true;
    max_range_pixel_ = int(param_max_range_ / msg_map_.info.resolution);
    ROS_INFO_NAMED(NAME_, "new global map set.");
  }
}

void UpdateCostmap::cfgCB(const gen_costmap2::GenCostmap2Config &config, uint32_t level)
{
  param_max_range_ = config.max_range;
  param_angle_resolution_ = config.angle_resolution;
  param_decay_ = config.decay;
  // TODO: fake_scan_t_ 更新
  if (map_set_)
  {
    max_range_pixel_ = int(param_max_range_ / msg_map_.info.resolution);
  }

  int new_num_rays = int(std::ceil(360. / param_angle_resolution_));
  if (new_num_rays != fake_num_rays_)
  {
    fake_num_rays_ = new_num_rays;
    // fake_scan_.assign(fake_num_rays_, std::numeric_limits<float>::max());
    fake_angle_resolution_ = ANGLE2RAD(param_angle_resolution_);
  }
  fake_scan_t_.clear();
  fake_scan_t_.resize(fake_num_rays_);
  for (int i = 0; i < fake_scan_t_.size(); ++i)
  {
    fake_scan_t_[i].rad = i * fake_angle_resolution_;
    fake_scan_t_[i].dist = param_max_range_;
    fake_scan_t_[i].x = param_max_range_ * std::cos(fake_scan_t_[i].rad);
    fake_scan_t_[i].y = param_max_range_ * std::sin(fake_scan_t_[i].rad);
  }

  ROS_INFO_NAMED(NAME_, "new config set.");
  ROS_INFO_NAMED(NAME_, "max_range: %lf", param_max_range_);
}

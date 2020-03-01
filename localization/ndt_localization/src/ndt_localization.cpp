/**
 * @brief 
 * 
 * @file ndt_localization.cpp
 * @author jyakaranda
 * @date 2018-09-20
 */

#include "ndt_localization/ndt_localization.h"

NDTLocalization::~NDTLocalization()
{
}

bool NDTLocalization::init()
{
  if (!initParams())
  {
    ROS_ERROR("error in NDTLocalization::init");
    return false;
  }

  if (param_init_pose_with_param)
  {
    init_pose_with_param();
  }

  if (param_debug_)
  {
    pub_predict_path_ = nh_.advertise<nav_msgs::Path>("/ndt/predict_path", 10);
  }
  pub_current_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/ndt/current_pose", 10);
  pub_path = nh_.advertise<nav_msgs::Path>("/debug/history_path", 10);
  pub_marker_loc_conf_ = nh_.advertise<visualization_msgs::Marker>("/ndt/loc_conf", 1);
  pub_marker_trans_prob_ = nh_.advertise<visualization_msgs::Marker>("/ndt/trans_prob", 1);
  pub_target_map = nh_.advertise<sensor_msgs::PointCloud2>("/map/cloud", 1);
  pub_ndt_score_ = nh_.advertise<std_msgs::Float64>("/ndt/ndt_score", 1);

  sub_initial_pose_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10, boost::bind(&NDTLocalization::initialPoseCB, this, _1));
  sub_point_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud", 5, boost::bind(&NDTLocalization::pointCloudCB, this, _1));

  if (param_use_odom_)
  {
    sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/odom/imu", 50, boost::bind(&NDTLocalization::odomCB, this, _1));
  }
  sub_imu_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data", 50, boost::bind(&NDTLocalization::imuCB, this, _1));

  while (!load_map(map_file))
  {
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  ROS_INFO("End init NDTLocalization");
  return true;
} // init params

bool NDTLocalization::initParams()
{

  pnh_.param<double>("tf_timeout", param_tf_timeout_, 0.05);
  pnh_.param<double>("odom_timeout", param_odom_timeout_, 1);
  pnh_.param<bool>("use_odom", param_use_odom_, true);
  pnh_.param<bool>("use_imu", param_use_imu_, true);

  pnh_.param<double>("predict_error_thresh", param_predict_error_thresh_, 0.5);
  // downsize source cloud
  pnh_.param<double>("voxel_leaf_size", voxel_leaf_size, 1.0);
  pnh_.param<double>("ndt_resolution", param_ndt_resolution_, 1.0);
  pnh_.param<int>("ndt_max_iterations", param_ndt_max_iterations_, 10);
  pnh_.param<double>("ndt_step_size", param_ndt_step_size_, 0.1);
  pnh_.param<double>("ndt_epsilon", param_ndt_epsilon_, 0.01);
  pnh_.param<int>("method_type", param_method_type_, 0);

  pnh_.param<bool>("if_init_pose_with_param", param_init_pose_with_param, true);

  // 更新局部target地图相关参数
  pnh_.param<bool>("use_local_target", use_local_target, true);
  pnh_.param<double>("target_map_radius", target_map_radius, 50.0);
  pnh_.param<double>("length_update_target_map", lengh_update_target_map, 3.0);
  pnh_.param<std::string>("global_map_file", map_file, "Confirm Location of Global Map.");

  pnh_.param<bool>("debug", param_debug_, false);
  debug_path.header.frame_id = "map";
  predict_path.header.frame_id = "map";

  pnh_.param<std::string>("latest_poses_path", latest_poses_path_, "");
  pnh_.param<std::string>("keypose_path", keyposes_path_, "");
  pnh_.param<std::string>("keyframes_path", keyframes_path_, "");
  pnh_.param<double>("surround_search_radius", surround_search_radius_, 40.);
  pnh_.param<int>("surround_search_num", surround_search_num_, 50);
  pnh_.param<double>("scan_period", scan_period_, 0.2);
  pnh_.param<double>("min_scan_range", param_min_scan_range_, 1.);
  pnh_.param<double>("max_scan_range", param_max_scan_range_, 70.);
  param_min_scan_range_ *= param_min_scan_range_;
  param_max_scan_range_ *= param_max_scan_range_;

  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

  if (param_method_type_ == METHOD_CUDA) // 定义GPU使用shared_ptr方式，因此需要初始化;
  {
    ROS_ERROR("error in method type");
    exit(-1);
  }

  Eigen::Translation3f tl_btol(0, 0, 1.0);
  double roll, pitch, yaw;
  if (!nh_.getParam("b2l_x", tl_btol.x()) || !nh_.getParam("b2l_y", tl_btol.y()) || !nh_.getParam("b2l_z", tl_btol.z()) || !nh_.getParam("b2l_roll", roll) || !nh_.getParam("b2l_pitch", pitch) || !nh_.getParam("b2l_yaw", yaw))
  {
    ROS_ERROR("Please set b2l!");
    return false;
  }
  roll = ANGLE2RAD(roll);
  pitch = ANGLE2RAD(pitch);
  yaw = ANGLE2RAD(yaw);
  Eigen::AngleAxisf rot_x_btol(0, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rot_y_btol(0, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(0, Eigen::Vector3f::UnitZ());
  tf_btol_ = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

  // current_map2odom
  current_map2odom_.setIdentity();

  fitness_score_ = 0.;
  pose_init_ = false;
  target_map_ptr.reset(new PointCloudT());
  pthread_mutex_init(&mutex_, NULL);

  if (param_use_odom_)
  {
    ROS_WARN("Use odom.");
  }
  else
  {
    ROS_WARN("Forbid odom");
  }
  if (use_local_target)
  {
    ROS_WARN("Use local target map");
  }
  else
  {
    ROS_WARN("Use global target map");
  }

  latest_poses_config_ = YAML::LoadFile(latest_poses_path_);
  localization_confidence_ = 1.;

  cloud_keyposes_.reset(new PointCloudT());

  kdtree_poses_.reset(new pcl::KdTreeFLANN<PointT>());

  imu_ptr_front_ = odom_ptr_front_ = 0;
  imu_ptr_last_ = odom_ptr_last_ = -1;
  imu_ptr_last_iter_ = odom_ptr_last_iter_ = 0;

  pre_pose_o_ = cur_pose_o_ = Eigen::Matrix4f::Zero();
  pre_pose_m_ = cur_pose_m_ = Eigen::Matrix4f::Identity();

  ROS_INFO("End init NDTLocalization");
  return true;
}

void NDTLocalization::init_pose_with_param()
{
  ROS_INFO("Init pose with param");
  pnh_.param<double>("init_x", initial_pose_.x, 0.0);
  pnh_.param<double>("init_y", initial_pose_.y, 0.0);
  pnh_.param<double>("init_z", initial_pose_.z, 0.0);
  pnh_.param<double>("init_roll", initial_pose_.roll, 0.0);
  pnh_.param<double>("init_pitch", initial_pose_.pitch, 0.0);
  pnh_.param<double>("init_yaw", initial_pose_.yaw, 0.0);

  cur_pose_m_.block<3, 3>(0, 0) = (Eigen::AngleAxisf(initial_pose_.yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(initial_pose_.pitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(initial_pose_.roll, Eigen::Vector3f::UnitX())).toRotationMatrix();
  cur_pose_m_(0, 3) = initial_pose_.x;
  cur_pose_m_(1, 3) = initial_pose_.y;
  cur_pose_m_(2, 3) = initial_pose_.z;
  pre_pose_m_ = cur_pose_m_;

  current_pose_imu_ = predict_pose_imu_ = pre_pose_ = current_pose_ = initial_pose_;
  pose_diff_.reset();
  added_pose = initial_pose_;
  pose_init_ = true;

  offset_imu_.reset();

  std::cout << "Initial pose with:" << std::endl;
  std::cout << "    init_x: " << initial_pose_.x << std::endl;
  std::cout << "    init_y: " << initial_pose_.y << std::endl;
  std::cout << "    init_z: " << initial_pose_.z << std::endl;
  std::cout << " init_roll: " << initial_pose_.roll << std::endl;
  std::cout << "init_pitch: " << initial_pose_.pitch << std::endl;
  std::cout << "  init_yaw: " << initial_pose_.yaw << std::endl;
  ROS_INFO("Current pose initialized.");
}

void NDTLocalization::initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  localization_confidence_ = 1.;
  if (msg->header.frame_id != "map")
  {
    ROS_WARN("Please initialize pose under /map frame.");
    pose_init_ = false;
    return;
  }
  geometryPose2Pose(msg->pose.pose, initial_pose_);

  cur_pose_m_.block<3, 3>(0, 0) = Eigen::Quaternionf(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();
  cur_pose_m_(0, 3) = msg->pose.pose.position.x;
  cur_pose_m_(1, 3) = msg->pose.pose.position.y;
  cur_pose_m_(2, 3) = msg->pose.pose.position.z;
  pre_pose_m_ = cur_pose_m_;

  current_pose_imu_ = predict_pose_imu_ = pre_pose_ = current_pose_ = initial_pose_;
  pose_diff_.reset();
  pose_init_ = true;

  offset_imu_.reset();

  std::cout << "Initial pose with:" << std::endl;
  std::cout << "    init_x: " << initial_pose_.x << std::endl;
  std::cout << "    init_y: " << initial_pose_.y << std::endl;
  std::cout << "    init_z: " << initial_pose_.z << std::endl;
  std::cout << " init_roll: " << initial_pose_.roll << std::endl;
  std::cout << "init_pitch: " << initial_pose_.pitch << std::endl;
  std::cout << "  init_yaw: " << initial_pose_.yaw << std::endl;
  ROS_INFO("-------------Current pose initialized.----------");
}

/**
 * @brief 1. caculate pdf(mean, covariance) for each voxel grid in model
 * 
 * @param msg better to be filtered map data.
 */
bool NDTLocalization::load_map(std::string map_file)
{

  if (!pose_init_)
  {
    ROS_WARN("initial pose not set, cannot update target_map");
    return false;
  }

  pcl::io::loadPCDFile(keyposes_path_, *cloud_keyposes_);
  PointCloudT::Ptr all_points(new PointCloudT());
  pcl::io::loadPCDFile(keyframes_path_, *all_points);

  // cloud_keyframes_.assign(cloud_keyposes_->points.size(), PointCloudT::Ptr(new PointCloudT()));
  cloud_keyframes_.resize(cloud_keyposes_->points.size());
  for (int i = 0; i < cloud_keyposes_->points.size(); ++i)
  {
    cloud_keyframes_[i] = PointCloudT::Ptr(new PointCloudT());
  }

  for (const auto &p : all_points->points)
  {
    cloud_keyframes_[int(p.intensity)]->points.push_back(p);
  }

  ROS_INFO("load %d keypose points, %d keyframe points", cloud_keyposes_->points.size(), all_points->points.size());

  kdtree_poses_->setInputCloud(cloud_keyposes_);

  if (use_local_target)
  {
    update_target_map(); // >>>>>>>>>>更新target地图
    ROS_WARN("(local)target map size: %d", target_map_ptr->points.size());
  }
  else
  {
    *target_map_ptr = *all_points;
    ROS_WARN("(global)target map size: %d", target_map_ptr->points.size());
  }

  // set NDT target
  pthread_mutex_lock(&mutex_);

  if (param_method_type_ == METHOD_CUDA)
  {
    ROS_ERROR("error in method type");
    exit(-1);
  }
  else if (param_method_type_ == METHOD_OMP)
  {
    ROS_ERROR("error in method type");
    exit(-1);
  }
  else if (param_method_type_ == METHOD_CPU)
  {
    cpu_ndt_.setResolution(param_ndt_resolution_);
    cpu_ndt_.setInputTarget(target_map_ptr);
    cpu_ndt_.setMaximumIterations(param_ndt_max_iterations_);
    cpu_ndt_.setStepSize(param_ndt_step_size_);
    cpu_ndt_.setTransformationEpsilon(param_ndt_epsilon_);

    PointCloudT::Ptr dummy_scan_ptr(new PointCloudT());
    PointT dummy_point;
    dummy_scan_ptr->push_back(dummy_point); // ????
    cpu_ndt_.setInputSource(dummy_scan_ptr);

    cpu_ndt_.align(Eigen::Matrix4f::Identity());
  }
  else
  {
    ROS_ERROR("error in method type");
    exit(-1);
  }

  // publish target_map
  sensor_msgs::PointCloud2::Ptr msg_target_map_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*target_map_ptr, *msg_target_map_ptr);
  msg_target_map_ptr->header.frame_id = "map";
  msg_target_map_ptr->header.stamp = ros::Time::now();
  pub_target_map.publish(*msg_target_map_ptr);

  map_init_ = true;
  pthread_mutex_unlock(&mutex_);
  ROS_INFO("Update model pc with %d points.", target_map_ptr->points.size());
  return true;
}

void NDTLocalization::odomCB(const nav_msgs::Odometry::ConstPtr &msg)
{

  odom_ptr_last_ = (odom_ptr_last_ + 1) % odom_que_length_;
  odom_que_[odom_ptr_last_] = *msg;
  if ((odom_ptr_last_ + 1) % odom_que_length_ == odom_ptr_front_)
  {
    odom_ptr_front_ = (odom_ptr_front_ + 1) % odom_que_length_;
  }

  tf_broadcaster_.sendTransform(tf::StampedTransform(current_map2odom_, msg->header.stamp, "map", "/odom"));
}

void NDTLocalization::imuCB(const sensor_msgs::ImuConstPtr &msg)
{
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(msg->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  float acc_x = msg->linear_acceleration.x + 9.81 * sin(pitch);
  float acc_y = msg->linear_acceleration.y - 9.81 * cos(pitch) * sin(roll);
  float acc_z = msg->linear_acceleration.z - 9.81 * cos(pitch) * cos(roll);

  imu_ptr_last_ = (imu_ptr_last_ + 1) % odom_que_length_;

  if ((imu_ptr_last_ + 1) % odom_que_length_ == imu_ptr_front_)
  {
    imu_ptr_front_ = (imu_ptr_front_ + 1) % odom_que_length_;
  }

  imu_time_[imu_ptr_last_] = msg->header.stamp.toSec();
  imu_roll_[imu_ptr_last_] = roll;
  imu_pitch_[imu_ptr_last_] = pitch;
  imu_yaw_[imu_ptr_last_] = yaw;
  imu_acc_x_[imu_ptr_last_] = acc_x;
  imu_acc_y_[imu_ptr_last_] = acc_y;
  imu_acc_z_[imu_ptr_last_] = acc_z;
  imu_angular_velo_x_[imu_ptr_last_] = msg->angular_velocity.x;
  imu_angular_velo_y_[imu_ptr_last_] = msg->angular_velocity.y;
  imu_angular_velo_z_[imu_ptr_last_] = msg->angular_velocity.z;

  // 转换到 imu 的全局坐标系中
  Eigen::Matrix3f rot = Eigen::Quaternionf(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z).toRotationMatrix();
  Eigen::Vector3f acc = rot * Eigen::Vector3f(acc_x, acc_y, acc_z);
  // TODO: lego_loam 里没有对角速度转换，是否需要尚且存疑
  // Eigen::Vector3f angular_velo = rot * Eigen::Vector3f(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  Eigen::Vector3f angular_velo(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  int imu_ptr_back = (imu_ptr_last_ - 1 + odom_que_length_) % odom_que_length_;
  double time_diff = imu_time_[imu_ptr_last_] - imu_time_[imu_ptr_back];
  if (time_diff < 1.)
  {
    imu_shift_x_[imu_ptr_last_] = imu_shift_x_[imu_ptr_back] + imu_velo_x_[imu_ptr_back] * time_diff + acc(0) * time_diff * time_diff * 0.5;
    imu_shift_y_[imu_ptr_last_] = imu_shift_y_[imu_ptr_back] + imu_velo_y_[imu_ptr_back] * time_diff + acc(1) * time_diff * time_diff * 0.5;
    imu_shift_z_[imu_ptr_last_] = imu_shift_z_[imu_ptr_back] + imu_velo_z_[imu_ptr_back] * time_diff + acc(2) * time_diff * time_diff * 0.5;

    imu_velo_x_[imu_ptr_last_] = imu_velo_x_[imu_ptr_back] + acc(0) * time_diff;
    imu_velo_y_[imu_ptr_last_] = imu_velo_y_[imu_ptr_back] + acc(1) * time_diff;
    imu_velo_z_[imu_ptr_last_] = imu_velo_z_[imu_ptr_back] + acc(2) * time_diff;

    imu_angular_rot_x_[imu_ptr_last_] = imu_angular_rot_x_[imu_ptr_back] + angular_velo(0) * time_diff;
    imu_angular_rot_y_[imu_ptr_last_] = imu_angular_rot_y_[imu_ptr_back] + angular_velo(1) * time_diff;
    imu_angular_rot_z_[imu_ptr_last_] = imu_angular_rot_z_[imu_ptr_back] + angular_velo(2) * time_diff;
  }
}

/**
 * @brief 参考 loam 的点云去运动畸变（基于匀速运动假设）
 * 
 */
void NDTLocalization::adjustDistortion(pcl::PointCloud<PointT>::Ptr &cloud, double scan_time)
{
  bool half_passed = false;
  int cloud_size = cloud->points.size();

  float start_ori = -std::atan2(cloud->points[0].y, cloud->points[0].x);
  float end_ori = -std::atan2(cloud->points[cloud_size - 1].y, cloud->points[cloud_size - 1].x);
  if (end_ori - start_ori > 3 * M_PI)
  {
    end_ori -= 2 * M_PI;
  }
  else if (end_ori - start_ori < M_PI)
  {
    end_ori += 2 * M_PI;
  }
  float ori_diff = end_ori - start_ori;

  Eigen::Vector3f rpy_start, shift_start, velo_start, rpy_cur, shift_cur, velo_cur;
  Eigen::Vector3f shift_from_start;
  Eigen::Matrix3f r_s_i, r_c;
  Eigen::Vector3f adjusted_p;
  float ori_h;
  for (int i = 0; i < cloud_size; ++i)
  {
    PointT &p = cloud->points[i];
    ori_h = -std::atan2(p.y, p.x);
    if (!half_passed)
    {
      if (ori_h < start_ori - M_PI * 0.5)
      {
        ori_h += 2 * M_PI;
      }
      else if (ori_h > start_ori + M_PI * 1.5)
      {
        ori_h -= 2 * M_PI;
      }

      if (ori_h - start_ori > M_PI)
      {
        half_passed = true;
      }
    }
    else
    {
      ori_h += 2 * M_PI;
      if (ori_h < end_ori - 1.5 * M_PI)
      {
        ori_h += 2 * M_PI;
      }
      else if (ori_h > end_ori + 0.5 * M_PI)
      {
        ori_h -= 2 * M_PI;
      }
    }

    float rel_time = (ori_h - start_ori) / ori_diff * scan_period_;

    if (imu_ptr_last_ > 0)
    {
      imu_ptr_front_ = imu_ptr_last_iter_;
      while (imu_ptr_front_ != imu_ptr_last_)
      {
        if (scan_time + rel_time > imu_time_[imu_ptr_front_])
        {
          break;
        }
        imu_ptr_front_ = (imu_ptr_front_ + 1) % odom_que_length_;
      }

      if (scan_time + rel_time > imu_time_[imu_ptr_front_])
      {
        rpy_cur(0) = imu_roll_[imu_ptr_front_];
        rpy_cur(1) = imu_pitch_[imu_ptr_front_];
        rpy_cur(2) = imu_yaw_[imu_ptr_front_];
        shift_cur(0) = imu_shift_x_[imu_ptr_front_];
        shift_cur(1) = imu_shift_y_[imu_ptr_front_];
        shift_cur(2) = imu_shift_z_[imu_ptr_front_];
        velo_cur(0) = imu_velo_x_[imu_ptr_front_];
        velo_cur(1) = imu_velo_y_[imu_ptr_front_];
        velo_cur(2) = imu_velo_z_[imu_ptr_front_];
      }
      else
      {
        int imu_ptr_back = (imu_ptr_front_ - 1 + odom_que_length_) % odom_que_length_;
        float ratio_front = (scan_time + rel_time - imu_time_[imu_ptr_back]) / (imu_time_[imu_ptr_front_] - imu_time_[imu_ptr_back]);
        float ratio_back = 1. - ratio_front;
        rpy_cur(0) = imu_roll_[imu_ptr_front_] * ratio_front + imu_roll_[imu_ptr_back] * ratio_back;
        rpy_cur(1) = imu_pitch_[imu_ptr_front_] * ratio_front + imu_pitch_[imu_ptr_back] * ratio_back;
        rpy_cur(2) = imu_yaw_[imu_ptr_front_] * ratio_front + imu_yaw_[imu_ptr_back] * ratio_back;
        shift_cur(0) = imu_shift_x_[imu_ptr_front_] * ratio_front + imu_shift_x_[imu_ptr_back] * ratio_back;
        shift_cur(1) = imu_shift_y_[imu_ptr_front_] * ratio_front + imu_shift_y_[imu_ptr_back] * ratio_back;
        shift_cur(2) = imu_shift_z_[imu_ptr_front_] * ratio_front + imu_shift_z_[imu_ptr_back] * ratio_back;
        velo_cur(0) = imu_velo_x_[imu_ptr_front_] * ratio_front + imu_velo_x_[imu_ptr_back] * ratio_back;
        velo_cur(1) = imu_velo_y_[imu_ptr_front_] * ratio_front + imu_velo_y_[imu_ptr_back] * ratio_back;
        velo_cur(2) = imu_velo_z_[imu_ptr_front_] * ratio_front + imu_velo_z_[imu_ptr_back] * ratio_back;
      }

      r_c = (Eigen::AngleAxisf(rpy_cur(2), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(rpy_cur(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(rpy_cur(0), Eigen::Vector3f::UnitX())).toRotationMatrix();

      if (i == 0)
      {
        rpy_start = rpy_cur;
        shift_start = shift_cur;
        velo_start = velo_cur;
        r_s_i = r_c.inverse();
      }
      else
      {
        shift_from_start = shift_cur - shift_start - velo_start * rel_time;
        adjusted_p = r_s_i * (r_c * Eigen::Vector3f(p.x, p.y, p.z) + shift_from_start);
        p.x = adjusted_p.x();
        p.y = adjusted_p.y();
        p.z = adjusted_p.z();
      }
    }
    imu_ptr_last_iter_ = imu_ptr_front_;
  }

  // if (pub_undistorted_pc_.getNumSubscribers() > 0)
  // {
  //   sensor_msgs::PointCloud2 msg;
  //   pcl::toROSMsg(*cloud, msg);
  //   msg.header.stamp.fromSec(scan_time);
  //   msg.header.frame_id = "/laser";
  //   pub_undistorted_pc_.publish(msg);
  // }
}

/** 
 * 1. get data points
 * 2. match data points to model points(map)
 * 2.1 caculate score function: put the point to corresponding pdf, and sum it up
 * 2.2 optimize transformation matrix(position) using Newton method until score function is converged
 */
void NDTLocalization::pointCloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  auto start_1 = std::chrono::system_clock::now();

  // TODO main function
  if (!map_init_ || !pose_init_)
  {
    ROS_WARN_STREAM("Cannot localize without given map and initial pose.");
    return;
  }

  PointCloudT::Ptr scan_ptr_temp(new PointCloudT());
  PointCloudT::Ptr scan_ptr(new PointCloudT());

  pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>());

  pcl::fromROSMsg(*msg, *tmp_cloud);
  // 点云转换到 /base_link 坐标系中处理
  pcl::transformPointCloud(*tmp_cloud, *scan_ptr, tf_btol_);

  if (param_use_imu_)
  {
    adjustDistortion(scan_ptr, msg->header.stamp.toSec());
  }

  tmp_cloud->clear();
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*tmp_cloud);
  scan_ptr->clear();
  float r;
  for (const auto &p : tmp_cloud->points)
  {
    r = p.x * p.x + p.y * p.y;
    if (r > param_min_scan_range_ && r < param_max_scan_range_)
    {
      scan_ptr->points.push_back(p);
    }
  }
  ROS_WARN("filtered size: %d", scan_ptr->points.size());

  PointCloudT::Ptr output_cloud(new PointCloudT());
  Eigen::Matrix4f init_guess;
  Eigen::Matrix4f final_tf;
  pose predict_ndt_pose;
  pose ndt_pose;

  // TODO predict_ndt_pose
  if (param_use_odom_)
  {
    if (odom_ptr_last_ == -1)
    {
      ROS_WARN("param_use_odom_ set true, but not odom msg received.");
      return;
    }

    int odom_ptr = odom_ptr_front_;
    while (odom_ptr != odom_ptr_last_)
    {
      if (odom_que_[odom_ptr].header.stamp > msg->header.stamp)
      {
        break;
      }
      odom_ptr = (odom_ptr + 1) % odom_que_length_;
    }

    if (pre_pose_o_(3, 3) == 0)
    {
      pre_pose_o_.setIdentity();
      pre_pose_o_.block<3, 3>(0, 0) = Eigen::Quaternionf(odom_que_[odom_ptr].pose.pose.orientation.w, odom_que_[odom_ptr].pose.pose.orientation.x, odom_que_[odom_ptr].pose.pose.orientation.y, odom_que_[odom_ptr].pose.pose.orientation.z).toRotationMatrix();
      pre_pose_o_(0, 3) = odom_que_[odom_ptr].pose.pose.position.x;
      pre_pose_o_(1, 3) = odom_que_[odom_ptr].pose.pose.position.y;
      pre_pose_o_(2, 3) = odom_que_[odom_ptr].pose.pose.position.z;
      cur_pose_o_ = pre_pose_o_;
      odom_ptr_front_ = odom_ptr; // 更新指针
      return;
    }
    else
    {
      pre_pose_o_ = cur_pose_o_;
      cur_pose_o_.block<3, 3>(0, 0) = Eigen::Quaternionf(odom_que_[odom_ptr].pose.pose.orientation.w, odom_que_[odom_ptr].pose.pose.orientation.x, odom_que_[odom_ptr].pose.pose.orientation.y, odom_que_[odom_ptr].pose.pose.orientation.z).toRotationMatrix();
      cur_pose_o_(0, 3) = odom_que_[odom_ptr].pose.pose.position.x;
      cur_pose_o_(1, 3) = odom_que_[odom_ptr].pose.pose.position.y;
      cur_pose_o_(2, 3) = odom_que_[odom_ptr].pose.pose.position.z;
      odom_ptr_front_ = odom_ptr; // 更新指针
    }

    Eigen::Quaternionf tmp_q(pre_pose_m_.block<3, 3>(0, 0));
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);
    // pre_pose_m_.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    cur_pose_m_ = pre_pose_m_ * pre_pose_o_.inverse() * cur_pose_o_; // 见笔记“预测当前位姿”

    tmp_q = Eigen::Quaternionf(cur_pose_m_.block<3, 3>(0, 0));
    tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);
    cur_pose_m_.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    std::cout << "initial guess: "
              << "(" << cur_pose_m_(0, 3) << ", " << cur_pose_m_(1, 3) << ", " << cur_pose_m_(2, 3) << ", " << roll
              << ", " << pitch << ", " << yaw << ")" << std::endl;

    predict_ndt_pose.x = cur_pose_m_(0, 3);
    predict_ndt_pose.y = cur_pose_m_(1, 3);
    predict_ndt_pose.z = cur_pose_m_(2, 3);
    predict_ndt_pose.roll = roll;
    predict_ndt_pose.pitch = pitch;
    predict_ndt_pose.yaw = yaw;
  }
  else
  {
    // TODO: predict pose
    predict_ndt_pose = pre_pose_;
    predict_ndt_pose += pose_diff_;
  }

  // predict_pose_ndt*odom is map_link->base_link; but init_guess matrix should be under the transform of map_link->laser_link   ===that's why <*tf_btol>
  init_guess = cur_pose_m_;

  ros::Time align_start, align_end, getFitnessScore_start, getFitnessScore_end;

  pthread_mutex_lock(&mutex_);
  if (param_method_type_ == METHOD_CUDA)
  {
    ROS_ERROR("error in method type");
    exit(-1);
  }
  else if (param_method_type_ == METHOD_OMP)
  {
    ROS_ERROR("error in method type");
    exit(-1);
  }
  else if (param_method_type_ == METHOD_CPU)
  {
    cpu_ndt_.setInputSource(scan_ptr);
    if (param_debug_)
    {
      ROS_INFO("Start align cpu");
    }
    align_start = ros::Time::now();
    cpu_ndt_.align(init_guess);
    align_end = ros::Time::now();

    final_tf = cpu_ndt_.getFinalTransformation();
    has_converged_ = cpu_ndt_.hasConverged();
    iteration_ = cpu_ndt_.getFinalNumIteration();
    trans_probability_ = cpu_ndt_.getTransformationProbability();

    getFitnessScore_start = ros::Time::now();
    fitness_score_ = cpu_ndt_.getFitnessScore();
    getFitnessScore_end = ros::Time::now();
  }
  else
  {
    ROS_ERROR("error in method type");
    exit(-1);
  }

  // if (param_debug_)
  // {
  //   ROS_INFO("NDT has converged(time: %.2f): %d, iterations: %d, fitness_score(time: %.2f): %f, trans_probability: %f", (align_end - align_start).toSec(), has_converged_, iteration_, (getFitnessScore_end - getFitnessScore_start).toSec(), fitness_score_, trans_probability_);
  // }

  pthread_mutex_unlock(&mutex_);

  cur_pose_m_ = final_tf;
  tf::Matrix3x3 mat_b;
  mat_b.setValue(static_cast<double>(cur_pose_m_(0, 0)), static_cast<double>(cur_pose_m_(0, 1)), static_cast<double>(cur_pose_m_(0, 2)),
                 static_cast<double>(cur_pose_m_(1, 0)), static_cast<double>(cur_pose_m_(1, 1)), static_cast<double>(cur_pose_m_(1, 2)),
                 static_cast<double>(cur_pose_m_(2, 0)), static_cast<double>(cur_pose_m_(2, 1)), static_cast<double>(cur_pose_m_(2, 2)));

  ndt_pose.x = cur_pose_m_(0, 3);
  ndt_pose.y = cur_pose_m_(1, 3);
  ndt_pose.z = cur_pose_m_(2, 3);
  mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw);

  predict_pose_error_ = std::sqrt(std::pow(ndt_pose.x - predict_ndt_pose.x, 2) + std::pow((ndt_pose.y - predict_ndt_pose.y), 2) +
                                  std::pow(ndt_pose.z - predict_ndt_pose.z, 2));
  // choose which pose to use as currnt pose  == between predict_ndt_pose  && ndt_pose(matching calculated)
  // if predict(guess) error is smaller than threshold , use it as current pose
  // else use ndt_pose as current pose

  bool use_predict_pose;
  if (predict_pose_error_ <= param_predict_error_thresh_ || std::abs(ndt_pose.yaw - predict_ndt_pose.yaw) > 0.3)
  {
    use_predict_pose = false;
  }
  else
  {
    ROS_WARN("use predict pose!");
    ROS_WARN("ndt_pose: (%.4f, %.4f, %.4f) (%.4f, %.4f, %.4f); predict_pose: (%.4f, %.4f, %.4f) (%.4f, %.4f, %.4f)",
             ndt_pose.x, ndt_pose.y, ndt_pose.z, ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, predict_ndt_pose.x, predict_ndt_pose.y, predict_ndt_pose.z, predict_ndt_pose.roll, predict_ndt_pose.pitch, predict_ndt_pose.yaw);
    use_predict_pose = true;
  }

  // use_predict_pose = false;

  if (!use_predict_pose)
  {
    current_pose_ = ndt_pose;
  }
  else
  {
    current_pose_ = predict_ndt_pose;
    cur_pose_m_.block<3, 3>(0, 0) = (Eigen::AngleAxisf(predict_ndt_pose.yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(predict_ndt_pose.pitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(predict_ndt_pose.roll, Eigen::Vector3f::UnitX())).toRotationMatrix();
    cur_pose_m_(0, 3) = predict_ndt_pose.x;
    cur_pose_m_(1, 3) = predict_ndt_pose.y;
    cur_pose_m_(2, 3) = predict_ndt_pose.z;
  }

  // 上面以获得确定的全局位置current_pose, 本处确定是否更新target地图
  double length;
  if (use_local_target)
  {
    length = hypot(current_pose_.x - added_pose.x, current_pose_.y - added_pose.y);
    if (length >= lengh_update_target_map)
    {
      update_target_map();

      pthread_mutex_lock(&mutex_);
      if (param_method_type_ == METHOD_CUDA)
      {
        ROS_ERROR("error in method type");
        exit(-1);
      }
      else if (param_method_type_ == METHOD_CPU)
      {
        auto start = std::chrono::system_clock::now();
        cpu_ndt_.setInputTarget(target_map_ptr);
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        ROS_WARN("update target: %fs", elapsed.count());
      }
      else if (param_method_type_ == METHOD_OMP)
      {
        ROS_ERROR("error in method type");
        exit(-1);
      }
      else
      {
        ROS_ERROR("error in method type");
        exit(-1);
      }
      pthread_mutex_unlock(&mutex_);

      added_pose = current_pose_;
    }
  }

  if (param_debug_)
  {
    std::cout << "    has converged: " << has_converged_ << std::endl;
    std::cout << "       align time: " << (align_end - align_start).toSec() << std::endl;
    std::cout << "       iterations: " << iteration_ << std::endl;
    std::cout << "    fitness score: " << fitness_score_ << std::endl;
    std::cout << "time to get score: " << (getFitnessScore_end - getFitnessScore_start).toSec() << std::endl;
    std::cout << "trans_probability: " << trans_probability_ << std::endl;
    std::cout << "current pose (x,y,z,roll,pitch,yaw): " << std::endl;
    std::cout << "(" << current_pose_.x << ", " << current_pose_.y << ", " << current_pose_.z << ", " << current_pose_.roll
              << ", " << current_pose_.pitch << ", " << current_pose_.yaw << ")" << std::endl;
    if (use_local_target)
    {
      std::cout << "shift to update localmap: " << length << std::endl;
    }
    std::cout << "------------------------------------------------" << std::endl;
  }

  pose2GeometryPose(msg_current_pose_.pose, current_pose_);
  msg_current_pose_.header.stamp = msg->header.stamp; // current pose is under "map_frame"
  msg_current_pose_.header.frame_id = "map";
  pub_current_pose_.publish(msg_current_pose_);

  std_msgs::Float64 msg_ndt_score;
  msg_ndt_score.data = fitness_score_;
  pub_ndt_score_.publish(msg_ndt_score);

  tf::Quaternion tmp_q;
  tmp_q.setRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw);
  tf::Transform tf_m2b(tmp_q, tf::Vector3(current_pose_.x, current_pose_.y, current_pose_.z));

  // publish map->odom using map->base and odom->base
  if (param_use_odom_)
  {
    tf::StampedTransform tf_o2b; // odom_link -> base_link
    tf_o2b.setOrigin(tf::Vector3(odom_que_[odom_ptr_front_].pose.pose.position.x, odom_que_[odom_ptr_front_].pose.pose.position.y, odom_que_[odom_ptr_front_].pose.pose.position.z));
    tf_o2b.setRotation(tf::Quaternion(odom_que_[odom_ptr_front_].pose.pose.orientation.x, odom_que_[odom_ptr_front_].pose.pose.orientation.y, odom_que_[odom_ptr_front_].pose.pose.orientation.z, odom_que_[odom_ptr_front_].pose.pose.orientation.w));
    current_map2odom_ = tf_m2b * tf_o2b.inverse();
    tf_broadcaster_.sendTransform(tf::StampedTransform(current_map2odom_, msg->header.stamp, "map", "/odom"));
  }
  else
  {
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_m2b, msg->header.stamp, "map", "/base_link"));
  }

  pose_diff_.x = current_pose_.x - pre_pose_.x;
  pose_diff_.y = current_pose_.y - pre_pose_.y;
  pose_diff_.z = 0;
  pose_diff_.yaw = current_pose_.yaw - pre_pose_.yaw;
  if (pose_diff_.yaw > M_PI)
  {
    pose_diff_.yaw -= 2 * M_PI;
  }
  else if (pose_diff_.yaw < M_PI)
  {
    pose_diff_.yaw += 2 * M_PI;
  }
  pre_pose_ = current_pose_;
  pre_pose_m_ = cur_pose_m_;

  if (param_debug_)
  {
    geometry_msgs::Vector3 scale;
    scale.x = 3.0 / (trans_probability_ + 0.1);
    scale.y = 3.0 * (fitness_score_ + 0.1);
    scale.z = 0.1;
    util::pubMarkerCylinder(pub_marker_loc_conf_, msg_current_pose_.pose, msg->header.stamp, "map", scale);

    std::stringstream ss;
    ss << std::fixed << std::setprecision(4) << "ndt_pose: (" << current_pose_.x << ", " << current_pose_.y << ", " << current_pose_.z << "; " << RAD2ANGLE(current_pose_.roll) << ", " << RAD2ANGLE(current_pose_.pitch) << ", " << RAD2ANGLE(current_pose_.yaw) << ")" << std::endl
       << "transform prob: " << trans_probability_ << std::endl
       << "ndt score: " << fitness_score_ << std::endl
       << "match time: " << (align_end - align_start).toSec() << "s" << std::endl
       << "iters: " << iteration_ << std::endl;
    geometry_msgs::Pose pose;
    pose.position.x = 0.;
    pose.position.z = 1.;
    pose.position.y = -20.;
    util::pubMarkerText(pub_marker_trans_prob_, pose, msg->header.stamp, "map", ss.str());
  }

  if (param_debug_)
  {
    geometry_msgs::PoseStamped msg_predict;
    pose2GeometryPose(msg_predict.pose, predict_ndt_pose);
    predict_path.poses.push_back(msg_predict);
    predict_path.header.stamp = msg->header.stamp;
    pub_predict_path_.publish(predict_path);
    pub_debug_path();
  }

  // TODO: 记录定位结果，用于定位失败重启，目前定位失败判断有问题
  std::map<std::string, double> cur_result;
  cur_result["time"] = msg->header.stamp.toSec();
  cur_result["fitness_score"] = fitness_score_;
  cur_result["x"] = current_pose_.x;
  cur_result["y"] = current_pose_.y;
  cur_result["z"] = current_pose_.z;
  cur_result["roll"] = current_pose_.roll;
  cur_result["pitch"] = current_pose_.pitch;
  cur_result["yaw"] = current_pose_.yaw;
  if (latest_results_.size() > latest_results_size_)
  {
    latest_results_.pop_back();
  }
  latest_results_.push_front(cur_result);

  if (fitness_score_ > 1.5)
  {
    localization_confidence_ *= 0.9;
  }
  else
  {
    localization_confidence_ = std::min(1., localization_confidence_ * 1.1);
  }

  // if (latest_poses_config_.IsDefined())
  // {
  //   latest_poses_config_["reboot_times"] = latest_poses_config_["reboot_times"].as<int>() + 1;
  //   latest_poses_config_["latest_poses"].push_back(cur_result);
  //   std::ofstream out(latest_poses_path_);
  //   out << latest_poses_config_;
  //   out.close();
  //   // TODO: 定位失败重启，判断条件修改，用户提示修改（包含向其他模块发送提示）
  //   if (localization_confidence_ < 0.6)
  //   {
  //     ROS_WARN("localization confidence too low, need to relocalize");
  //     if (latest_poses_config_["reboot_times"].as<int>() > 5)
  //     {
  //       ROS_ERROR("too many times reboot. please give correct initial pose");
  //       // exit(-1);
  //     }
  //     else
  //     {
  //       pose recent_pose;
  //       while (!latest_results_.empty())
  //       {
  //         const auto &tmp = latest_results_.front();
  //         latest_results_.pop_front();
  //         if (tmp.at("fitness_score") < 1.3)
  //         {
  //           recent_pose.x = tmp.at("x");
  //           recent_pose.y = tmp.at("y");
  //           recent_pose.z = tmp.at("z");
  //           recent_pose.roll = tmp.at("roll");
  //           recent_pose.pitch = tmp.at("pitch");
  //           recent_pose.yaw = tmp.at("yaw");
  //           ROS_WARN("reset robot pose: (%f, %f, %f, %f, %f, %f)", recent_pose.x, recent_pose.y, recent_pose.z, recent_pose.roll, recent_pose.pitch, recent_pose.yaw);
  //           break;
  //         }
  //       }

  //       // geometry_msgs::PoseWithCovarianceStampedPtr relocate_pose(new geometry_msgs::PoseWithCovarianceStamped);
  //       // pose2GeometryPose(relocate_pose->pose.pose, recent_pose);
  //       // initialPoseCB(relocate_pose);
  //     }
  //   }
  // }

  auto end_1 = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_1 = end_1 - start_1;
  ROS_INFO("predict time elapsed: %fs", elapsed_1.count());
}

void NDTLocalization::update_target_map()
{
  target_map_ptr->clear();

  PointT cur_pose;
  cur_pose.x = current_pose_.x;
  cur_pose.y = current_pose_.y;
  cur_pose.z = current_pose_.z;

  kdtree_poses_->radiusSearch(cur_pose, surround_search_radius_, point_search_idx_, point_search_dist_);

  for (int i = 0; i < point_search_idx_.size(); ++i)
  {
    *target_map_ptr += *(cloud_keyframes_[point_search_idx_[i]]);
  }

  // publish target_map
  sensor_msgs::PointCloud2::Ptr msg_target_map_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*target_map_ptr, *msg_target_map_ptr);
  msg_target_map_ptr->header.frame_id = "map";
  msg_target_map_ptr->header.stamp = ros::Time::now();
  pub_target_map.publish(*msg_target_map_ptr);

  ROS_WARN("update local map with %d points", target_map_ptr->points.size());
}

void NDTLocalization::pub_debug_path()
{
  geometry_msgs::PoseStamped p;
  p.pose.position.x = current_pose_.x;
  p.pose.position.y = current_pose_.y;
  p.pose.position.z = current_pose_.z;

  Eigen::AngleAxisd roll_angle(current_pose_.roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(current_pose_.pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(current_pose_.yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = roll_angle * pitch_angle * yaw_angle;
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  p.pose.orientation.w = q.w();

  debug_path.poses.push_back(p);
  pub_path.publish(debug_path);
}

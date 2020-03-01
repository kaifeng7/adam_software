#ifndef __NDT_LOCALZATION__
#define __NDT_LOCALZATION__

#include <ros/ros.h>
#include <ros/duration.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <array>
#include <deque>
#include <map>
#include <sstream>
#include <pthread.h>
#include <chrono>
#include <boost/thread/thread.hpp>

#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ndt_cpu/NormalDistributionsTransform.h>

#include <yaml-cpp/yaml.h>

#include "user_protocol.h"
#include "utils.hpp"

#define METHOD_PCL 0
#define METHOD_CUDA 1
#define METHOD_OMP 2
#define METHOD_CPU 3

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class NDTLocalization
{
public:
  NDTLocalization(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh), target_map_ptr(new PointCloudT())
  {
  }
  ~NDTLocalization();
  /**
   * @brief Initialize. 
   * 
   * @return true 
   * @return false 
   */
  bool init();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;

  ros::Publisher pub_predict_path_;
  ros::Publisher pub_current_pose_;
  geometry_msgs::PoseStamped msg_current_pose_;
  ros::Publisher pub_marker_loc_conf_;
  ros::Publisher pub_marker_trans_prob_;
  ros::Publisher pub_ndt_score_;

  ros::Publisher pub_target_map;

  ros::Subscriber sub_odom_;

  ros::Subscriber sub_map_;
  ros::Subscriber sub_initial_pose_;
  pose initial_pose_; // under map frame
  ros::Subscriber sub_point_cloud_;
  ros::Subscriber sub_imu_;
  sensor_msgs::Imu msg_imu_;

  PointCloudT data_pc_; // 定义要加载的全局地图
  PointCloudT::Ptr target_map_ptr;

  PointCloudT::Ptr cloud_keyposes_;
  std::vector<PointCloudT::Ptr> cloud_keyframes_;

  pcl::KdTreeFLANN<PointT>::Ptr kdtree_poses_;

  std::vector<int> point_search_idx_;
  std::vector<float> point_search_dist_;

  std::string keyposes_path_;
  std::string keyframes_path_;
  double surround_search_radius_;
  int surround_search_num_;
  double scan_period_;

  static const int odom_que_length_ = 300;
  std::array<nav_msgs::Odometry, odom_que_length_> odom_que_;
  int odom_ptr_front_, odom_ptr_last_, odom_ptr_last_iter_;

  int imu_ptr_front_, imu_ptr_last_, imu_ptr_last_iter_;
  Eigen::Vector3f rpy_cur_, velo_xyz_cur_, shift_xyz_cur_;
  Eigen::Vector3f rpy_start_, velo_xyz_start_, shift_xyz_start_;
  Eigen::Vector3f shift_from_start_;

  std::array<double, odom_que_length_> imu_time_;
  std::array<float, odom_que_length_> imu_roll_;
  std::array<float, odom_que_length_> imu_pitch_;
  std::array<float, odom_que_length_> imu_yaw_;

  std::array<float, odom_que_length_> imu_acc_x_;
  std::array<float, odom_que_length_> imu_acc_y_;
  std::array<float, odom_que_length_> imu_acc_z_;
  std::array<float, odom_que_length_> imu_velo_x_;
  std::array<float, odom_que_length_> imu_velo_y_;
  std::array<float, odom_que_length_> imu_velo_z_;
  std::array<float, odom_que_length_> imu_shift_x_;
  std::array<float, odom_que_length_> imu_shift_y_;
  std::array<float, odom_que_length_> imu_shift_z_;

  std::array<float, odom_que_length_> imu_angular_velo_x_;
  std::array<float, odom_que_length_> imu_angular_velo_y_;
  std::array<float, odom_que_length_> imu_angular_velo_z_;
  std::array<float, odom_que_length_> imu_angular_rot_x_;
  std::array<float, odom_que_length_> imu_angular_rot_y_;
  std::array<float, odom_que_length_> imu_angular_rot_z_;

  // publish debug_path
  nav_msgs::Path debug_path;
  nav_msgs::Path predict_path;
  ros::Publisher pub_path;

  pose added_pose; // 更新局部target地图相关
  bool use_local_target;
  double lengh_update_target_map;
  double target_map_radius;

  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  double voxel_leaf_size;
  double param_min_scan_range_;
  double param_max_scan_range_;

  Eigen::Matrix4f pre_pose_o_;
  Eigen::Matrix4f cur_pose_o_;
  Eigen::Matrix4f pre_pose_m_;
  Eigen::Matrix4f cur_pose_m_;

  pose current_pose_;
  pose pre_pose_;
  pose pose_diff_;
  pose current_pose_imu_;
  pose predict_pose_imu_;
  ros::Time pre_odom_time_;
  pose offset_imu_;
  // ros::Time pre_imu_time_;
  Eigen::Matrix4f tf_btol_;
  tf::Transform current_map2odom_;

  bool pose_init_;
  bool map_init_;
  pthread_mutex_t mutex_;

  cpu::NormalDistributionsTransform<PointT, PointT> cpu_ndt_;
  bool has_converged_;
  double fitness_score_;
  double trans_probability_;
  int iteration_;
  double predict_pose_error_;

  double param_tf_timeout_;
  double param_odom_timeout_;
  bool param_use_odom_;
  bool param_use_imu_;
  double param_predict_error_thresh_;

  double param_ndt_resolution_;
  int param_ndt_max_iterations_;
  double param_ndt_step_size_;
  double param_ndt_epsilon_;
  int param_method_type_;

  // debug use
  bool param_debug_;

  std::string map_file;

  double localization_confidence_;
  int latest_results_size_;
  std::deque<std::map<std::string, double>> latest_results_;

  std::string latest_poses_path_;
  YAML::Node latest_poses_config_;

  bool initParams();

  bool load_map(std::string map_file);

  void adjustDistortion(pcl::PointCloud<PointT>::Ptr &cloud, double scan_time);

  /**
   * @brief Save motion data to get a rough pose estimation to give NDT-matching a initial transformation matrix.
   * 
   * @param msg 
   */
  void odomCB(const nav_msgs::Odometry::ConstPtr &msg);

  /**
   * @brief Set a rough pose estimation by manual. 
   * 
   * @param msg 
   */
  void initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  /**
   * @brief Get measured data points, estimate current pose using 3D-NDT-matching.
   * 
   * @param msg 
   */
  void pointCloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg);

  void imuCB(const sensor_msgs::ImuConstPtr &msg);

  static bool pubMarkerText(const ros::Publisher pub, const geometry_msgs::Pose pose, const ros::Time stamp, const std::string frame, const std::string text);

  static bool pubMarkerCylinder(const ros::Publisher pub, const geometry_msgs::Pose pose, const ros::Time stamp, const std::string frame, const geometry_msgs::Vector3 scale);

  static bool pubMarkerCube(const ros::Publisher pub, const geometry_msgs::Pose pose, const ros::Time stamp, const std::string frame, const geometry_msgs::Vector3 scale);

  void init_pose_with_param();
  double init_x, init_y, init_z;
  double init_roll, init_pitch, init_yaw;
  bool param_init_pose_with_param;

  void update_target_map();

  void pub_debug_path();
};

#endif
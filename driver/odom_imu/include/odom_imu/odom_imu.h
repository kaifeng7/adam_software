#ifndef __ODOM_IMU__
#define __ODOM_IMU__

#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "user_protocol.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

class OdomImu
{
public:
  OdomImu(const ros::NodeHandle nh, const ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
  {
  }
  ~OdomImu();
  bool init();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;
  tf::StampedTransform tf_btoi_;
  bool tf_init_;

  ros::Publisher pub_odom_;
  nav_msgs::Odometry msg_odom_;
  ros::Publisher pub_predict_;
  ros::Publisher pub_mesurement_;
  ros::Publisher pub_update_;

  ros::Subscriber sub_imu_;
  sensor_msgs::Imu msg_imu_;
  ros::Subscriber sub_encoder_;
  geometry_msgs::TwistStamped msg_encoder_;
  geometry_msgs::TwistStamped cur_vel_;
  ros::Subscriber sub_mode_stat_;
  std_msgs::Int32 msg_mode_stat_;
  ros::Subscriber sub_cur_pose_;

  ros::Time pre_time_;
  bool first_imu_;
  bool first_encoder_;

  pose pre_pose_;
  pose current_pose_;
  double current_vel_x_;
  double current_vel_y_;
  double current_vel_z_;

  double param_max_interval_;
  double param_angle_vel_sensitive_;
  double param_linear_vel_sensitive_;
  std::string param_base_frame_;
  std::string param_odom_frame_;

  // EKF 相关
  Eigen::Vector3f mu_pre, mu_cur, z_cur;
  Eigen::Matrix3f Sigma_pre, Sigma_cur;
  Eigen::Matrix3f G_cur, R_cur;
  Eigen::Matrix3f H_cur, Q_cur;
  Eigen::Matrix3f K_cur;

  tf::Transform current_map2odom_;

  void imuCB(const sensor_msgs::Imu::ConstPtr &msg);
  void encoderCB(const geometry_msgs::TwistStamped::ConstPtr &msg);
  void modeStatCB(const std_msgs::Int32::ConstPtr &msg);
  void poseCB(const geometry_msgs::PoseStampedConstPtr &msg);
};

#endif
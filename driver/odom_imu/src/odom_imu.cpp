#include "odom_imu/odom_imu.h"

OdomImu::~OdomImu()
{
}

bool OdomImu::init()
{
  ROS_INFO("Start init OdomImu.");

  first_imu_ = true;
  first_encoder_ = true;
  tf_init_ = false;

  pnh_.param<double>("max_interval", param_max_interval_, 1.0);
  pnh_.param<double>("angle_vel_sensitive", param_angle_vel_sensitive_, 0.001);
  pnh_.param<double>("linear_vel_sensitive", param_linear_vel_sensitive_, 0.001);
  pnh_.param<std::string>("base_frame", param_base_frame_, std::string("/base_link"));
  pnh_.param<std::string>("odom_frame", param_odom_frame_, std::string("/odom"));

  msg_mode_stat_.data = 1;

  mu_cur = mu_pre = z_cur = Eigen::Vector3f::Zero();
  R_cur = Q_cur = Sigma_pre = Eigen::Matrix3f::Zero();
  R_cur(0, 0) = 0.00001;
  R_cur(1, 1) = 0.00001;
  R_cur(2, 2) = 0.05;
  Q_cur(0, 0) = Q_cur(1, 1) = 0.1;
  Q_cur(2, 2) = 0.01;
  Sigma_pre(0, 0) = Sigma_pre(1, 1) = 0.1;
  Sigma_pre(2, 2) = 0.1;
  Sigma_pre(0, 1) = Sigma_pre(1, 0) = 0.01;
  Sigma_cur = Sigma_pre;
  H_cur = Eigen::Matrix3f::Identity();

  current_map2odom_.setIdentity();

  sub_imu_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data", 500, boost::bind(&OdomImu::imuCB, this, _1));
  sub_encoder_ = nh_.subscribe<geometry_msgs::TwistStamped>("/wheel_circles", 100, boost::bind(&OdomImu::encoderCB, this, _1));
  sub_mode_stat_ = nh_.subscribe<std_msgs::Int32>("/mode_stat", 1, boost::bind(&OdomImu::modeStatCB, this, _1));
  sub_cur_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/ndt/current_pose", 5, boost::bind(&OdomImu::poseCB, this, _1));
  pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/odom/imu", 10);
  pub_predict_ = nh_.advertise<nav_msgs::Odometry>("/predict", 1);
  pub_mesurement_ = nh_.advertise<nav_msgs::Odometry>("/measurement", 1);
  pub_update_ = nh_.advertise<nav_msgs::Odometry>("/update", 1);

  ROS_INFO("End init OdomImu.");

  return true;
}

void OdomImu::encoderCB(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  if (first_encoder_)
  {
    first_encoder_ = false;
    ROS_INFO("Received first encoder.");
  }
  cur_vel_ = *msg;
  if ((msg_mode_stat_.data < 0 && cur_vel_.twist.linear.x > 0) || (msg_mode_stat_.data > 0 && cur_vel_.twist.linear.x < 0))
  {
    cur_vel_.twist.linear.x *= -1;
  }
}

void OdomImu::imuCB(const sensor_msgs::Imu::ConstPtr &msg)
{
  if (first_imu_)
  {
    first_imu_ = false;
    pre_time_ = msg->header.stamp;
    return;
  }
  else if (first_encoder_)
  {
    ROS_WARN("Have not received encoder, not publish odom");
    return;
  }

  double diff_time = (msg->header.stamp - pre_time_).toSec();
  if (diff_time > param_max_interval_)
  {
    ROS_WARN("Long time waiting for next imu msg. Igore this msg.");
    pre_time_ = msg->header.stamp;
    return;
  }
  else if ((msg->header.stamp - cur_vel_.header.stamp).toSec() > param_max_interval_)
  {
    ROS_WARN("Long time waiting for encoder msg update. Igore this msg.");
    return;
  }

  // 进行一个低通滤波
  // double angle_vel_z = round(msg->angular_velocity.z / param_angle_vel_sensitive_) * param_angle_vel_sensitive_;

  static double pre_angle_vel_z = round(msg->angular_velocity.z * 1000) * 0.001;
  double angle_vel_z = round(msg->angular_velocity.z * 1000) * 0.001;
  if (diff_time > 0.0001)
  {
    double acc_angle_vel = (angle_vel_z - pre_angle_vel_z) / diff_time;
    if (std::abs(acc_angle_vel) > 3.14)
    {
      angle_vel_z = 0.;
    }
  }
  pre_angle_vel_z = angle_vel_z;
  double vel = round(cur_vel_.twist.linear.x * 1000) * 0.001;

  mu_pre = mu_cur;
  Sigma_pre = Sigma_cur;

  // EKF: 1. predict
  if (std::abs(angle_vel_z) < 0.001)
  {
    mu_cur = mu_pre + Eigen::Vector3f(vel * diff_time * cos(mu_pre(2)), vel * diff_time * sin(mu_pre(2)), 0);
    G_cur = Eigen::Matrix3f::Identity();
    G_cur(0, 2) = -vel * diff_time * sin(mu_pre(2));
    G_cur(1, 2) = vel * diff_time * cos(mu_pre(2));
    Sigma_cur = G_cur * Sigma_pre * G_cur.transpose() + R_cur;
  }
  else
  {
    double r = vel / angle_vel_z;
    double delta_yaw = angle_vel_z * diff_time;
    mu_cur = mu_pre + Eigen::Vector3f(r * (-sin(mu_pre(2)) + sin(mu_pre(2) + delta_yaw)),
                                      r * (cos(mu_pre(2)) - cos(mu_pre(2) + delta_yaw)),
                                      delta_yaw);
    G_cur = Eigen::Matrix3f::Identity();
    G_cur(0, 2) = r * (cos(mu_pre(2)) - cos(mu_pre(2) + delta_yaw));
    G_cur(1, 2) = r * (sin(mu_pre(2)) - sin(mu_pre(2) + delta_yaw));
    Sigma_cur = G_cur * Sigma_pre * G_cur.transpose() + R_cur;
  }

  if (mu_cur(2) > 2 * M_PI)
  {
    mu_cur(2) -= 2 * M_PI;
  }
  else if (mu_cur(2) < 0)
  {
    mu_cur(2) += 2 * M_PI;
  }

  std::cout << "predict <--------->" << std::endl;
  std::cout << "mu_: " << mu_cur << "; Sigma_: " << Sigma_cur << std::endl;

  tf::Quaternion tmp_q;
  tmp_q.setRPY(0, 0, mu_cur(2));
  tf::Transform tf_o2b(tmp_q, tf::Vector3(mu_cur(0), mu_cur(1), 0));
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_o2b, msg->header.stamp, "/odom", "/base_link"));

  pre_time_ = msg->header.stamp;

  msg_odom_.header.stamp = msg->header.stamp;
  msg_odom_.header.frame_id = param_odom_frame_;
  msg_odom_.child_frame_id = param_base_frame_;
  tf::pointTFToMsg(tf_o2b.getOrigin(), msg_odom_.pose.pose.position);
  tf::quaternionTFToMsg(tf_o2b.getRotation(), msg_odom_.pose.pose.orientation);

  msg_odom_.twist.twist.angular.z = angle_vel_z;
  msg_odom_.twist.twist.linear.x = cur_vel_.twist.linear.x;
  pub_odom_.publish(msg_odom_);

  nav_msgs::Odometry msg_predict;
  msg_predict.header = msg_odom_.header;
  msg_predict.pose.pose = msg_odom_.pose.pose;
  pub_predict_.publish(msg_predict);

  // tf_broadcaster_.sendTransform(tf::StampedTransform(current_map2odom_, msg->header.stamp, "map", "/odom"));
}

void OdomImu::modeStatCB(const std_msgs::Int32::ConstPtr &msg)
{
  ROS_INFO("OdomImu: mode changed. pre: %d, cur: %d", msg_mode_stat_.data, msg->data);
  msg_mode_stat_.data = msg->data;
}

void OdomImu::poseCB(const geometry_msgs::PoseStampedConstPtr &msg)
{
  // tf::StampedTransform tf_m2o;
  // tf_m2o.setIdentity();
  // try
  // {
  //   tf_listener_.waitForTransform("map", "/odom", ros::Time(0), ros::Duration(0.25));
  //   tf_listener_.lookupTransform("map", "/odom", ros::Time(0), tf_m2o);
  // }
  // catch (tf::TransformException &ex)
  // {
  //   ROS_ERROR("Transform error in poseCB: %s", ex.what());
  //   return;
  // }

  tf::Pose m2b;
  tf::poseMsgToTF(msg->pose, m2b);
  tf::Pose o2b = current_map2odom_.inverse() * m2b;

  nav_msgs::Odometry msg_measurement, msg_update;
  msg_measurement.header.stamp = msg_update.header.stamp = msg->header.stamp;
  msg_measurement.header.frame_id = msg_update.header.frame_id = "/odom";
  tf::poseTFToMsg(o2b, msg_measurement.pose.pose);

  double roll, pitch, yaw;
  tf::Matrix3x3(o2b.getRotation()).getRPY(roll, pitch, yaw);

  z_cur(0) = o2b.getOrigin().getX();
  z_cur(1) = o2b.getOrigin().getY();
  z_cur(2) = yaw;

  Eigen::Matrix3f tmp_m = H_cur * Sigma_cur * H_cur.transpose() + Q_cur;

  // EKF: 2. update
  K_cur = Sigma_cur * H_cur.transpose() * tmp_m.inverse();
  mu_cur = mu_cur + K_cur * (z_cur - mu_cur);
  Sigma_cur = (Eigen::Matrix3f::Identity() - K_cur * H_cur) * Sigma_cur;

  if (mu_cur(2) > 2 * M_PI)
  {
    mu_cur(2) -= 2 * M_PI;
  }
  else if (mu_cur(2) < 0)
  {
    mu_cur(2) += 2 * M_PI;
  }

  std::cout << "update <--------->" << std::endl;
  std::cout << "z: " << z_cur << " mu_cur: " << mu_cur << "; Sigma: " << Sigma_cur << std::endl;

  o2b.setOrigin(tf::Vector3(mu_cur(0), mu_cur(1), o2b.getOrigin().getZ()));
  tf::Quaternion tmp_q1;
  tmp_q1.setRPY(roll, pitch, mu_cur(2));
  o2b.setRotation(tmp_q1);

  ROS_INFO("o2b: %f, %f, %f; %f, %f, %f, %f", o2b.getOrigin().getX(), o2b.getOrigin().getY(), o2b.getOrigin().getZ(), o2b.getRotation().getX(), o2b.getRotation().getY(), o2b.getRotation().getZ(), o2b.getRotation().getW());

  tf::Pose tmp_p = m2b * o2b.inverse();

  ROS_INFO("tmp_p: %f, %f, %f; %f, %f, %f, %f", tmp_p.getOrigin().getX(), tmp_p.getOrigin().getY(), tmp_p.getOrigin().getZ(), tmp_p.getRotation().getX(), tmp_p.getRotation().getY(), tmp_p.getRotation().getZ(), tmp_p.getRotation().getW());

  current_map2odom_.setOrigin(tmp_p.getOrigin());
  current_map2odom_.setRotation(tmp_p.getRotation());

  tf::poseTFToMsg(o2b, msg_update.pose.pose);

  pub_mesurement_.publish(msg_measurement);
  pub_update_.publish(msg_update);

  // tf_broadcaster_.sendTransform(tf::StampedTransform(current_map2odom_, msg->header.stamp, "map", "/odom"));
}
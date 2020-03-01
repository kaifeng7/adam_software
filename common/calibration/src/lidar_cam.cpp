/**
 * @file lidar_cam.cpp
 * @author heng zhang (you@domain.com)
 * @brief 激光雷达和摄像头的数据融合
 * @version 0.1
 * @date 2018-12-27
 * 
 * @copyright Copyright (c) 2018
 * 
 */
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamic_reconfigure/server.h>
#include <calibration/LidarCamConfig.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

#define TAG "calibration"

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

image_transport::Subscriber sub_img_;
ros::Subscriber sub_cam_info_;
ros::Subscriber sub_pc_;
ros::Publisher pub_fused_pc_;
image_transport::Publisher pub_fused_img_;

double param_l2c_x_;
double param_l2c_y_;
double param_l2c_z_;
double param_l2c_roll_;
double param_l2c_pitch_;
double param_l2c_yaw_;
std::vector<double> param_l2c_extrinsic_;
int param_point_size_;
bool param_use_tf_; // 是否使用 /tf 获取 tf_l2c_
std::string param_frame_img_;

bool cam_info_ok_;
bool img_ok_;
cv::Mat cur_img_;                 // 校正后的图像
cv::Mat cam_intrinsincs_;         // 相机内参
cv::Mat distortion_coefficients_; // 畸变系数
double fx_, fy_, cx_, cy_;
int img_width_, img_height_;

tf::Transform tf_l2c_;

PointT transformPoint(const PointT src, tf::Transform tf)
{
  tf::Vector3 p(src.x, src.y, src.z);
  tf::Vector3 p_t = tf * p;
  PointT val(src.intensity);
  val.x = p_t.getX();
  val.y = p_t.getY();
  val.z = p_t.getZ();
  return val;
}

void camInfoCB(const sensor_msgs::CameraInfoConstPtr &msg)
{
  cam_intrinsincs_ = cv::Mat(3, 3, CV_64F);
  for (int row = 0; row < 3; row++)
  {
    for (int col = 0; col < 3; col++)
    {
      cam_intrinsincs_.at<double>(row, col) = msg->K[row * 3 + col];
    }
  }

  distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
  for (int col = 0; col < 5; col++)
  {
    distortion_coefficients_.at<double>(col) = msg->D[col];
  }

  fx_ = static_cast<double>(msg->P[0]);
  fy_ = static_cast<double>(msg->P[5]);
  cx_ = static_cast<double>(msg->P[2]);
  cy_ = static_cast<double>(msg->P[6]);

  cam_info_ok_ = true;
  sub_cam_info_.shutdown();
  ROS_INFO_NAMED(TAG, "CameraInfo set.");
}

void imgCB(const sensor_msgs::ImageConstPtr &msg)
{
  if (!cam_info_ok_)
  {
    ROS_WARN_NAMED(TAG, "waiting for camera info.");
    return;
  }
  param_frame_img_ = msg->header.frame_id;

  cv::Mat in_img;
  try
  {
    in_img = cv_bridge::toCvShare(msg, "bgr8")->image;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR_NAMED(TAG, "%s", e.what());
    return;
  }
  cv::undistort(in_img, cur_img_, cam_intrinsincs_, distortion_coefficients_);

  img_width_ = cur_img_.cols;
  img_height_ = cur_img_.rows;
  img_ok_ = true;
}

void pcCB(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  if (!img_ok_)
  {
    ROS_WARN_NAMED(TAG, "waiting for image.");
    return;
  }

  if (param_use_tf_)
  {
    tf::TransformListener tf_l;
    tf::StampedTransform tf_tmp;
    tf_l.lookupTransform(msg->header.frame_id, param_frame_img_, ros::Time(0), tf_tmp);
    tf_l2c_.setRotation(tf_tmp.getRotation());
    tf_l2c_.setOrigin(tf_tmp.getOrigin());
  }

  PointCloudT pc_src;
  pcl::fromROSMsg(*msg, pc_src);
  pcl::PointCloud<pcl::PointXYZRGB> pc_out;

  cv::Mat img_out = cur_img_.clone();
  // #pragma omp for
  for (int i = 0; i < pc_src.size(); i++)
  {
    PointT tmp = transformPoint(pc_src[i], tf_l2c_.inverse());
    int u = int(tmp.x * fx_ / tmp.z + cx_);
    int v = int(tmp.y * fy_ / tmp.z + cy_);
    if (u >= 0 && u < img_width_ && v >= 0 && v < img_height_ && tmp.z > 0)
    {
      pcl::PointXYZRGB tmp_p;
      tmp_p.x = pc_src[i].x;
      tmp_p.y = pc_src[i].y;
      tmp_p.z = pc_src[i].z;
      cv::Vec3b pixel = cur_img_.at<cv::Vec3b>(v, u);
      tmp_p.r = pixel.val[2];
      tmp_p.g = pixel.val[1];
      tmp_p.b = pixel.val[0];
      // img_out.at<cv::Vec3b>(v, u) = cv::Vec3b(0, 0, 255);
      for (int j = u - param_point_size_ / 2; j < u + param_point_size_ / 2 + 1; ++j)
      {
        for (int k = v - param_point_size_ / 2; k < v + param_point_size_ / 2 + 1; ++k)
        {
          if (j >= 0 && j < img_width_ && k >= 0 && k < img_height_)
          {
            img_out.at<cv::Vec3b>(k, j) = cv::Vec3b(0, 0, 255 * tmp.intensity / 100);
          }
        }
      }
      pc_out.push_back(tmp_p);
    }
  }
  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(pc_out, msg_out);
  msg_out.header = msg->header;
  pub_fused_pc_.publish(msg_out);

  sensor_msgs::ImagePtr msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_out).toImageMsg();
  msg_img->header.stamp = msg->header.stamp;
  msg_img->header.frame_id = "/camera_link";
  pub_fused_img_.publish(msg_img);
}

void cfgCB(const calibration::LidarCamConfig &config, uint32_t level)
{
  if (!param_use_tf_)
  {
    param_l2c_x_ = config.l2c_x;
    param_l2c_y_ = config.l2c_y;
    param_l2c_z_ = config.l2c_z;
    param_l2c_roll_ = config.l2c_roll / 180. * 3.1415926;
    param_l2c_pitch_ = config.l2c_pitch / 180. * 3.1415926;
    param_l2c_yaw_ = config.l2c_yaw / 180. * 3.1415926;
    tf_l2c_.setOrigin(tf::Vector3(param_l2c_x_, param_l2c_y_, param_l2c_z_));
    tf::Quaternion q;
    q.setRPY(param_l2c_roll_, param_l2c_pitch_, param_l2c_yaw_);
    tf_l2c_.setRotation(q);
  }

  param_point_size_ = config.point_size;
  ROS_INFO_NAMED(TAG, "new l2c config set.");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_cam");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  image_transport::ImageTransport it(nh);

  sub_img_ = it.subscribe("/usb_cam/image_raw", 1, imgCB);
  sub_cam_info_ = nh.subscribe("/usb_cam/camera_info", 1, camInfoCB);
  sub_pc_ = nh.subscribe("/lslidar_point_cloud", 1, &pcCB);
  pub_fused_pc_ = nh.advertise<sensor_msgs::PointCloud2>("/fused_pc", 1);
  pub_fused_img_ = it.advertise("/fused_img", 1);

  dynamic_reconfigure::Server<calibration::LidarCamConfig> cfg_server;
  dynamic_reconfigure::Server<calibration::LidarCamConfig>::CallbackType cfg_callback = boost::bind(&cfgCB, _1, _2);
  cfg_server.setCallback(cfg_callback);
  pnh.param("use_tf", param_use_tf_, false);
  if (!param_use_tf_ && pnh.getParam("lidar2camera", param_l2c_extrinsic_))
  {
    tf_l2c_.setOrigin(tf::Vector3(param_l2c_extrinsic_[3], param_l2c_extrinsic_[7], param_l2c_extrinsic_[11]));
    tf::Matrix3x3 mat;
    mat.setValue(param_l2c_extrinsic_[0], param_l2c_extrinsic_[1], param_l2c_extrinsic_[2],
                 param_l2c_extrinsic_[4], param_l2c_extrinsic_[5], param_l2c_extrinsic_[6],
                 param_l2c_extrinsic_[8], param_l2c_extrinsic_[9], param_l2c_extrinsic_[10]);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf_l2c_.setRotation(q);
    ROS_INFO("camera to lidar extrinsic set: ");
    for (int i = 0; i < param_l2c_extrinsic_.size(); ++i)
    {
      std::cout << " " << param_l2c_extrinsic_[i];
      if (i % 3 == 0)
      {
        std::cout << std::endl;
      }
    }
  }
  ros::spin();
  return 0;
}

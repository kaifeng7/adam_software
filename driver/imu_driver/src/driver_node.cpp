/**
 * @file driver_node.cpp
 * @author heng zhang (you@domain.com)
 * @brief HW579 imu 驱动，包含校准功能
 * @version 0.1
 * @date 2019-05-10
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include "imu_driver/itg3205.h"
#include "imu_driver/adxl345.h"
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include "imu_driver/RawImu.h"

#include <string>
#include <map>

bool is_calibrated_;
bool perform_calibration_;
std::map<std::string, double> gyro_bias_;
std::map<std::string, double> acc_bias_;
int calibration_samples_;

static const double GRAVITY = -9.81; // [m/s/s]

bool calibrateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_WARN("service Calibrating accelerometer and gyroscope, make sure robot is stationary and level.");
  perform_calibration_ = true;
  acc_bias_["x"] = 0;
  acc_bias_["y"] = 0;
  acc_bias_["z"] = 0;
  gyro_bias_["x"] = 0;
  gyro_bias_["y"] = 0;
  gyro_bias_["z"] = 0;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_driver_node");
  ROS_INFO("Start driver_node.");
  ITG3205 gyro_node(1);
  if (!gyro_node.openITG3205())
  {
    ROS_ERROR("Failed to open ITG3205.");
    return -1;
  }
  if (!gyro_node.init())
  {
    ROS_ERROR("Failed to init ITG3205");
    return -1;
  }

  ADXL345 accel_node(1, 16);
  if (!accel_node.openADXL345())
  {
    ROS_ERROR("Failed to open ADXL345");
    return -1;
  }
  if (!accel_node.init())
  {
    ROS_ERROR("Failed to init ADXL345");
    return -1;
  }

  ROS_INFO("HW579 init ok.");

  ros::NodeHandle nh, pnh("~");
  ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 50);
  ros::ServiceServer imu_cal_srv = nh.advertiseService("imu/calibrate_imu", calibrateCallback);
  bool debug;
  nh.param<bool>("debug", debug, false);
  if (!pnh.getParam("acc_bias", acc_bias_) || !pnh.getParam("gyro_bias", gyro_bias_))
  {
    ROS_WARN("IMU calibration NOT found.");
  }
  else
  {
    ROS_INFO("IMU calibration found.");
    is_calibrated_ = true;
  }
  pnh.param<int>("calibration_samples", calibration_samples_, 500);
  pnh.param<bool>("perform_calibration", perform_calibration_, true);

  sensor_msgs::Imu msg_imu;

  ros::Duration duration(0.01);
  int count = 0;
  while (ros::ok())
  {
//    double cur_gyro = ros::Time::now().toSec();
    imu_data gyro = gyro_node.getGyro();
    usleep(5);
//    double end_gyro = ros::Time::now().toSec();
    imu_data accel = accel_node.getAccel();
//    double end_acc = ros::Time::now().toSec();
//    printf("gyro: %.5f, acc: %.5f", end_gyro - cur_gyro, end_acc - end_gyro);
 
    if (perform_calibration_ || !is_calibrated_)
    {
      ROS_WARN_ONCE("Calibrating accelerometer and gyroscope, make sure robot is stationary and level.");

      static int taken_samples;
      if (taken_samples < calibration_samples_)
      {
        acc_bias_["x"] += accel.accel.x;
        acc_bias_["y"] += accel.accel.y;
        acc_bias_["z"] += accel.accel.z;

        gyro_bias_["x"] += gyro.gyro.x;
        gyro_bias_["y"] += gyro.gyro.y;
        gyro_bias_["z"] += gyro.gyro.z;

        ++taken_samples;
      }
      else
      {
        acc_bias_["x"] /= calibration_samples_;
        acc_bias_["y"] /= calibration_samples_;
        acc_bias_["z"] = acc_bias_["z"] / calibration_samples_ + GRAVITY;

        gyro_bias_["x"] /= calibration_samples_;
        gyro_bias_["y"] /= calibration_samples_;
        gyro_bias_["z"] /= calibration_samples_;

        ROS_INFO("Calibrating accelerometer and gyroscope complete.");
        ROS_INFO("Bias values can be saved for reuse.");
        ROS_INFO("Accelerometer: x:%f, y:%f, z:%f", acc_bias_["x"], acc_bias_["y"], acc_bias_["z"]);
        ROS_INFO("Gyroscope: x:%f, y:%f, z:%f", gyro_bias_["x"], gyro_bias_["y"], gyro_bias_["z"]);

        pnh.setParam("acc_bias", acc_bias_);
        pnh.setParam("gryo_bias", gyro_bias_);

        is_calibrated_ = true;
        perform_calibration_ = false;
        taken_samples = 0;
      }
      continue;
    }

    msg_imu.header.frame_id = "/imu_link";
    msg_imu.header.stamp = ros::Time::now();

    msg_imu.linear_acceleration.x = accel.accel.x - acc_bias_["x"];
    msg_imu.linear_acceleration.y = accel.accel.y - acc_bias_["y"];
    msg_imu.linear_acceleration.z = accel.accel.z - acc_bias_["z"];
    msg_imu.angular_velocity.x = gyro.gyro.x - gyro_bias_["x"];
    msg_imu.angular_velocity.y = gyro.gyro.y - gyro_bias_["y"];
    msg_imu.angular_velocity.z = gyro.gyro.z - gyro_bias_["z"];
    // TODO: 角速度和加速度的协方差矩阵没有设置，应该查芯片手册补上
    pub_imu.publish(msg_imu);
//static double last = ros::Time::now().toSec();
//double cur = ros::Time::now().toSec();
//printf("duration: %lf\n", cur - last);
//last = cur;

    duration.sleep();
    ros::spinOnce();
  }

  return 0;
}

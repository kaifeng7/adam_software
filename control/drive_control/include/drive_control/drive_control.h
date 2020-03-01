#ifndef __DRIVE_CONTORL__
#define __DRIVE_CONTORL__

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <pthread.h>
#include <algorithm>
#include <functional>
#include <vector>

#include <ros/ros.h>
#include <ros/duration.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Int32.h>
#include <adam_msgs/ControlCommandStamped.h>
#include <adam_msgs/VehicleCmd.h>

#include "user_protocol.h"
#include "utils.hpp"

#define __CONST_SOCKADDR_ARG const struct sockaddr*

class DriveControl
{
public:
  DriveControl(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh) {}
  ~DriveControl() {}
  bool run();

private:
  bool init();
  void odomCB(const nav_msgs::Odometry::ConstPtr &msg);
  void cmdCB(const adam_msgs::VehicleCmd::ConstPtr &msg);
  bool openUDPPort();
  bool changeMode(int target_mode);
  bool stopCar();
  // double brake(int cur_mode, double acc, double cur_vel, double target_vel);
  // double accel(int cur_mode, double acc, double cur_vel, double target_vel);
  bool sendPacket(int __fd, const void *__buf, size_t __n,
                  int __flags, __CONST_SOCKADDR_ARG __addr,
                  socklen_t __addr_len);
  bool sendPwm(double pwm1, double pwm2, double duration);

  nav_msgs::Odometry cur_odom_;
  adam_msgs::VehicleCmd cur_cmd_;
  std::vector<geometry_msgs::Point32> pwm1_vf_;
  std::vector<geometry_msgs::Point32> pwm1_vb_;
  double cur_pwm1_;
  double cur_pwm2_;
  double delta_pwm1_;
  double delta_pwm2_;
  PacketData cur_packet_;
  ros::Duration udp_duration_;
  int cur_mode_; // 用于记录当前车辆档位状态：大于 0 为前进，小于 0 为倒退，初始化为前进
  pthread_mutex_t mutex_;

  // Socket relate variables
  in_addr device_ip_;
  int socket_fd_;
  struct sockaddr_in mcu_addr_;
  socklen_t mcu_addr_len_;
  in_addr MCU_ip_;
  int MCU_UDP_PORT_NUMBER_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_odom_;
  nav_msgs::Odometry msg_odom_;
  ros::Subscriber sub_cmd_;
  adam_msgs::ControlCommand msg_cmd_;
  ros::Publisher pub_mode_stat_;
  std_msgs::Int32 msg_mode_stat_;
  ros::Publisher pub_applied_ctrl_;
  adam_msgs::ControlCommandStamped msg_applied_ctrl_;

  double param_pwm1_ml_;  // 电机中位最小值
  double param_pwm1_mh_;  // 电机中位最大值
  double param_pwm1_max_; // 电机最大 pwm
  double param_pwm1_min_; // 电机最小 pwm
  double param_pwm1_min_f_;
  double param_pwm1_min_b_;
  std::string param_pwm1_vf_string_;
  std::string param_pwm1_vb_string_;
  double param_pwm1_max_diff_; // 发送的包之间的 pwm 最大差值，防止剧烈抖动
  double param_pwm2_ml_;       // 舵机中位最小值
  double param_pwm2_mh_;       // 舵机中位最大值
  double param_pwm2_max_;      // 舵机最大 pwm
  double param_pwm2_min_;      // 舵机最小 pwm
  double param_pwm2_max_diff_;
  int param_pwm_duration_; // stm32 发送 pwm 波持续时长 msec

  // double param_wheel_radius_; // 车轮半径

  std::string param_device_ip_str_; // 设备 ip
  int param_udp_port_num_;          // 设备端口号
  std::string param_mcu_ip_str_;    // mcu ip
  int param_mcu_udp_port_;          // mcu 端口号

  double param_udp_wait_;        // udp 包的发送间隔 msec
  double param_default_acc_;     // 默认加速度，未在 ControlCommand 中指定加速度时使用该加速度 1m/s^2
  double param_default_ang_vel_; // 默认转角速度，同上 rad/s，注意：不是角速度，而是转角速度。角速度是指车辆朝向的变化率，转角速度是方向盘（前轮转角）的变化率。
  double param_v1_;
  double param_v1_pwm_; // v1 前进速度对应的 pwm 波，用于计算前进加速度与 pwm 变化率的关系
  double param_v2_;
  double param_v2_pwm_; // v2 前进速度对应的 pwm 波
  double param_v3_;
  double param_v3_pwm_; // v3 倒退速度对应的 pwm 波，用于计算倒退加速度与 pwm 变化率的关系
  double param_v4_;
  double param_v4_pwm_;          // v4 倒退速度对应的 pwm
  double param_delta_ang_r_;     // 右转角度变化
  double param_delta_ang_r_pwm_; // delta_ang_r 对应的 pwm 变化，用于计算转角速度与 pwm 变化率的关系
  double param_delta_ang_l_;     // 左转角度变化
  double param_delta_ang_l_pwm_; // delta_ang_l 对应的 pwm 变化
  bool param_human_drive_;

  bool use_speed_feedback_; // 是否使用速度反馈
  double Kp_;
  double Ki_;
  double Kd_;
  double err_[3];
  double pwm1_stop_;
  double speed_feedback_duration_;
  };

#endif
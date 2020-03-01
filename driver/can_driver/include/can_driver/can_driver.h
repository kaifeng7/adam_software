/**
 * @file can_driver.h
 * @author heng zhang (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2018-12-20
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#ifndef __DDL_CANDRIVER__
#define __DDL_CANDRIVER__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <ros/ros.h>
#include <adam_msgs/VehicleCmd.h>

#include "utils.hpp"

class CanDriver
{
public:
  CanDriver(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~CanDriver();
  bool run();

private:
  std::string TAG_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  int can_sr_; // can raw socket, 接收
  int can_st_; // can raw socket, 发送
  int required_mtu_;
  int mtu_;
  int enable_canfd_;
  struct sockaddr_can addr_;
  struct canfd_frame frame_;
  struct ifreq ifr_;
  canid_t r_can_addr_; // 接收的 can 地址
  canid_t t_can_addr_; // 发送的 can 地址

  std::string param_r_canid_;    // 接收的 canid
  std::string param_t_canid_;    // 发送的 canid
  std::string param_r_can_addr_; // 接收的 can 地址
  std::string param_t_can_addr_; // 发送的 can 地址，存储的是 16 进制地址，如 (0x)203

  bool init();
  bool openCan();

  void controlCmdCB(const adam_msgs::VehicleCmdConstPtr &msg);
  int cmdMsg2CanFrame(const adam_msgs::VehicleCmdConstPtr &msg, struct canfd_frame *cf);
};

#endif
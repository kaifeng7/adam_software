/**
 * @file ethernet_viewer.h
 * @author heng zhang
 * @brief 
 * @version 0.1
 * @date 2018-11-26
 * 
 * @copyright Copyright (c) 2018
 * 
 */
#ifndef __ETHERNET_VIEWER__
#define __ETHERNET_VIEWER__

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <arpa/inet.h>

#include <fcntl.h>
#include <sys/file.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>

#include <pthread.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "ethernet_viewer/packet_data.h"

using namespace std;

class EthernetViewer
{
public:
  EthernetViewer() = default;
  EthernetViewer(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~EthernetViewer(){};
  bool run();

private:
  bool init();
  bool openUDPPort();
  void pcCB(const sensor_msgs::PointCloud2ConstPtr &msg);
  void imageCB(const sensor_msgs::ImageConstPtr &msg);
  bool sendPacket(uint8_t* data, const int &n, const uint8_t type);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_pc_;
  sensor_msgs::PointCloud2 msg_pc_;
  ros::Subscriber sub_image_;

  string TAG_;

  in_addr device_ip_;
  int socket_fd_;
  struct sockaddr_in mcu_addr_;
  socklen_t mcu_addr_len_;
  in_addr MCU_ip_;
  int MCU_UDP_PORT_NUMBER_;
  pthread_mutex_t mutex_;
  uint8_t *m_buf_;

  std::string param_device_ip_str_; // 设备 ip
  int param_udp_port_num_;          // 设备端口号
  std::string param_mcu_ip_str_;    // mcu ip
  int param_mcu_udp_port_;          // mcu 端口号

  const int slice_len_ = 1200;
};

#endif
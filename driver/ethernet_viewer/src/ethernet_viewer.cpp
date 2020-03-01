/**
 * @file ethernet_viewer.cpp
 * @author heng zhang (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2018-11-26
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#include "ethernet_viewer/ethernet_viewer.h"

EthernetViewer::EthernetViewer(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh), TAG_("EthernetViewer")
{
  if (!init())
  {
    ROS_ERROR_NAMED(TAG_, "init error.");
    ros::shutdown();
  }

  if (!openUDPPort())
  {
    ROS_ERROR_NAMED(TAG_, "open port error.");
  }
}

bool EthernetViewer::init()
{
  ROS_INFO_NAMED(TAG_, "init start.");

  sub_pc_ = nh_.subscribe<sensor_msgs::PointCloud2>("/viewer/pointcloud", 1, boost::bind(&EthernetViewer::pcCB, this, _1));
  sub_image_ = nh_.subscribe<sensor_msgs::Image>("/viewer/image", 1, boost::bind(&EthernetViewer::imageCB, this, _1));

  pnh_.param<std::string>("device_ip", param_device_ip_str_, "192.168.137.2");
  pnh_.param<int>("udp_port", param_udp_port_num_, 8888);
  pnh_.param<std::string>("mcu_ip", param_mcu_ip_str_, "192.168.137.3");
  pnh_.param<int>("mcu_udp_port", param_mcu_udp_port_, 8888);

  pthread_mutex_init(&mutex_, NULL);

  ROS_INFO_NAMED(TAG_, "init end.");
  return true;
}

bool EthernetViewer::openUDPPort()
{
  socket_fd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ == -1)
  {
    perror("socket");
    return false;
  }
  memset(&mcu_addr_, 0, sizeof(mcu_addr_));
  mcu_addr_.sin_family = AF_INET;
  mcu_addr_.sin_addr.s_addr = inet_addr(param_mcu_ip_str_.c_str());
  mcu_addr_.sin_port = htons(param_mcu_udp_port_);
  mcu_addr_len_ = sizeof(mcu_addr_);

  sockaddr_in my_addr;                           // my address information
  memset(&my_addr, 0, sizeof(my_addr));          // initialize to zeros
  my_addr.sin_family = AF_INET;                  // host byte order
  my_addr.sin_port = htons(param_mcu_udp_port_); // short, in network byte order
  ROS_INFO_STREAM("Opening mcu UDP socket: port " << param_mcu_udp_port_);
  my_addr.sin_addr.s_addr = INADDR_ANY; // automatically fill in my IP

  if (bind(socket_fd_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
  {
    perror("bind"); // TODO: ROS_ERROR errno
    return false;
  }

  if (fcntl(socket_fd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
    perror("non-block");
    return false;
  }

  return true;
}

void EthernetViewer::pcCB(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  ROS_INFO_NAMED(TAG_, "test start");
  packet_data::PointCloud pc(msg);
  // ROS_INFO("construct");
  pthread_mutex_lock(&mutex_);

  m_buf_ = new uint8_t[pc.size * 16 + 49];
  uint32_t temp;
  // type
  m_buf_[0] = pc.type;
  //timestamp
  ROS_INFO("pc_timestamp=%ld", pc.header.stamp);
  std::memcpy(&temp, ((uint8_t*)&pc.header.stamp) + 4, 4);
  temp = htonl(temp);
  std::memcpy(m_buf_ + 1, &temp, 4);
  std::memcpy(&temp, (uint8_t*)&pc.header.stamp, 4);
  temp = htonl(temp);
  std::memcpy(m_buf_ + 5, &temp, 4);
  //frame_id
  ROS_INFO("pc_frame_id=%s", pc.header.frame_id);
  std::memcpy(m_buf_ + 9, pc.header.frame_id, 32);
  //seq
  temp = htonl(pc.header.seq);
  ROS_INFO("pc_seq=%d", pc.header.seq);
  std::memcpy(m_buf_ + 41, &temp, 4);
  //size
  temp = htonl(pc.size);
  ROS_INFO("pc_size=%d", pc.size);
  std::memcpy(m_buf_ + 45, &temp, 4);
  //points
  ROS_INFO("points");
  for (int i = 0; i < pc.size; i++)
  {
    std::memcpy(&temp, &pc.points[i].x, 4);
    temp = htonl(temp);
    std::memcpy(m_buf_ + 49 + 16 * i, &temp, 4);

    std::memcpy(&temp, &pc.points[i].y, 4);
    temp = htonl(temp);
    std::memcpy(m_buf_ + 49 + 16 * i + 4, &temp, 4);

    std::memcpy(&temp, &pc.points[i].z, 4);
    temp = htonl(temp);
    std::memcpy(m_buf_ + 49 + 16 * i + 8, &temp, 4);
    
    std::memcpy(&temp, &pc.points[i].i, 4);
    temp = htonl(temp);
    std::memcpy(m_buf_ + 49 + 16 * i + 12, &temp, 4);
  }


  // TODO: 分片
  // 分片交给sendPacket完成
  int len = sizeof(pc.type) + sizeof(pc.header) + sizeof(pc.size) + sizeof(packet_data::Point) * pc.size;
  ROS_INFO("send packet start");
  // sendPacket(m_buf_, len, packet_data::T_POINT_CLOUD);
  sendPacket(m_buf_, len, packet_data::T_POINT_CLOUD);
  delete[] m_buf_;
  pthread_mutex_unlock(&mutex_);
  ROS_INFO_NAMED(TAG_, "test end");
}

void EthernetViewer::imageCB(const sensor_msgs::ImageConstPtr &msg)
{
  ROS_INFO("imageCB");
  packet_data::CameraImage c_img(msg);

  pthread_mutex_lock(&mutex_);
  // m_buf_ = (uint8_t *)&c_img;
  //除数据以外的长度
  int headLen = sizeof(c_img.type) + sizeof(c_img.header) + sizeof(c_img.height) + sizeof(c_img.width) + sizeof(c_img.byte_depth) +
            sizeof(c_img.num_channels) + sizeof(c_img.is_bigendian) + sizeof(c_img.step);

  m_buf_ = new uint8_t[1000000 + headLen];
  uint32_t temp;
  // type
  m_buf_[0] = c_img.type;
  //timestamp
  ROS_INFO("img_timestamp=%ld", c_img.header.stamp);
  std::memcpy(&temp, ((uint8_t*)&c_img.header.stamp) + 4, 4);
  temp = htonl(temp);
  std::memcpy(m_buf_ + 1, &temp, 4);
  std::memcpy(&temp, (uint8_t*)&c_img.header.stamp, 4);
  temp = htonl(temp);
  std::memcpy(m_buf_ + 5, &temp, 4);
  //frame_id
  ROS_INFO("img_frameid=%s", c_img.header.frame_id);
  std::memcpy(m_buf_ + 9, c_img.header.frame_id, 32);
  //seq
  ROS_INFO("img_seq=%d", c_img.header.seq);
  temp = htonl(c_img.header.seq);
  std::memcpy(m_buf_ + 41, &temp, 4);
  //height
  ROS_INFO("img_height=%d", c_img.height);
  temp = htonl(c_img.height);
  std::memcpy(m_buf_ + 45, &temp, 4);
  //width
  ROS_INFO("img_width=%d", c_img.width);
  temp = htonl(c_img.width);
  std::memcpy(m_buf_ + 49, &temp, 4);
  //byte_depth
  ROS_INFO("img_bytedepth=%d", c_img.byte_depth);
  temp = htonl(c_img.byte_depth);
  std::memcpy(m_buf_ + 53, &temp, 4);
  //num_channels
  ROS_INFO("img_num_channels=%d", c_img.num_channels);
  temp = htonl(c_img.num_channels);
  std::memcpy(m_buf_ + 57, &temp, 4);
  //is_bigendian
  ROS_INFO("img_is_bigendian=%d", (int)c_img.is_bigendian);
  std::memcpy(m_buf_ + 61, &c_img.is_bigendian, 1);
  //step
  ROS_INFO("img_step=%d", c_img.step);
  temp = htonl(c_img.step);
  std::memcpy(m_buf_ + 62, &temp, 4);
  //data
  std::memcpy(m_buf_ +66, c_img.data, c_img.step * c_img.height);

  // TODO
  int len =  headLen + sizeof(uint8_t) * c_img.height * c_img.step;
  
  sendPacket(m_buf_, len, packet_data::T_CAMERA_IMAGE);
  delete[] m_buf_;
  pthread_mutex_unlock(&mutex_);
}

bool EthernetViewer::sendPacket(uint8_t *data, const int &len, const uint8_t type)
{
    ROS_INFO("sending. offset");

  uint8_t *buf = data;
  int offset = 0; // 数据片偏移，单位byte
  
  ROS_INFO("sending. offset: %d, len: %d", offset, len);

  // packet_data::PointCloud *pc = (packet_data::PointCloud *)data;

  uint8_t sendbuf[1209]; //4+4+1200

  //数据总长
  sendbuf[0] = (len >> 24) & 0xff;
  sendbuf[1] = (len >> 16) & 0xff;
  sendbuf[2] = (len >> 8) & 0xff;
  sendbuf[3] = len & 0xff;
  
  uint8_t *sendbuf_ptr = sendbuf;
  // *(int*)sendbuf_ptr = len;
  //  ROS_INFO("sending. offset: %d, len: %d", offset, len);

  bool flag = true;

  while (offset <= len)
  {
    //数据片偏移
    sendbuf[4] = (offset >> 24) & 0xff;
    sendbuf[5] = (offset >> 16) & 0xff;
    sendbuf[6] = (offset >> 8) & 0xff;
    sendbuf[7] = offset & 0xff;

    try
    {
      std::memcpy(sendbuf_ptr + 8, data + offset, slice_len_);

      ssize_t nbytes = sendto(socket_fd_, sendbuf, slice_len_ + 8, 0, (struct sockaddr *)&mcu_addr_, mcu_addr_len_);

      usleep(1500);
      offset += slice_len_;

      if ((size_t)nbytes != (slice_len_ + 8))
      {
        ROS_INFO("sending. offset: %d, len: %d", offset, len);
        ROS_ERROR_NAMED(TAG_, "cannot send data to viewer!\n nbytes=%d", (int)nbytes);
        ROS_ERROR_NAMED(TAG_, "Errno: %d", errno);
        return false;
      }
    }
    catch (exception &e)
    {
      string str(e.what());
      ROS_ERROR_NAMED(TAG_, "str");
    }
  }
}

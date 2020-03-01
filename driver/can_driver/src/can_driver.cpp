/**
 * @file can_driver.cpp
 * @author heng zhang (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2018-12-20
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#include "can_driver/can_driver.h"

CanDriver::CanDriver(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
{
  if (!init())
  {
    ROS_ERROR_NAMED(TAG_, "Error in init.");
  }
}

CanDriver::~CanDriver()
{
  close(can_st_);
}

bool CanDriver::run()
{
  ros::Subscriber sub_ctrlCmd = nh_.subscribe<adam_msgs::VehicleCmd>("/ctrl_cmd", 1, boost::bind(&CanDriver::controlCmdCB, this, _1));

  ros::spin();
  return true;
}

bool CanDriver::init()
{
  pnh_.param<std::string>("r_canid", param_r_canid_, "can0");
  pnh_.param<std::string>("t_canid", param_t_canid_, "can1");
  pnh_.param<std::string>("r_can_addr", param_r_can_addr_, "");
  pnh_.param<std::string>("t_can_addr", param_t_can_addr_, "203"); // 0x203

  if (param_t_can_addr_.size() != 3)
  {
    ROS_ERROR_NAMED(TAG_, "t_can_addr not supported.");
    return false;
  }
  else
  {
    unsigned char tmp;
    for (int i = 0; i < 3; ++i)
    {
      auto asc2nibble = [](char c) -> char {
        if (c >= '0' && c <= '9')
          return c - '0';
        else if (c >= 'a' && c <= 'f')
          return c - 'a';
        else if (c >= 'A' && c <= 'F')
          return c - 'A';
        return 16;
      };
      if ((tmp = asc2nibble(param_t_can_addr_.at(i))) > 0x0F)
      {
        ROS_ERROR_NAMED(TAG_, "Error t_can_addr: %s", param_t_can_addr_.c_str());
        return false;
      }
      t_can_addr_ |= (tmp << (2 - i) * 4);
    }
  }

  enable_canfd_ = 1;

  if (!openCan())
  {
    ROS_ERROR_NAMED(TAG_, "Error in openCan.");
    return false;
  }

  return true;
}

bool CanDriver::openCan()
{
  ROS_INFO_NAMED(TAG_, "Start openCan.");
  if ((can_st_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    ROS_ERROR_NAMED(TAG_, "Error in open can_st.");
    return false;
  }
  if ((can_sr_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    ROS_ERROR_NAMED(TAG_, "Error in open can_sr.");
    return false;
  }

  strncpy(ifr_.ifr_name, param_t_canid_.c_str(), IFNAMSIZ - 1);
  ifr_.ifr_name[IFNAMSIZ - 1] = '\0';
  ifr_.ifr_ifindex = if_nametoindex(ifr_.ifr_name);
  if (!ifr_.ifr_ifindex)
  {
    ROS_ERROR_NAMED(TAG_, "if_nametoindex");
    return false;
  }

  memset(&addr_, 0, sizeof(addr_));
  addr_.can_family = AF_CAN;
  addr_.can_ifindex = ifr_.ifr_ifindex;

  /* disable default receive filter on this RAW socket */
  /* This is obsolete as we do not read from the socket at all, but for */
  /* this reason we can remove the receive list in the Kernel to save a */
  /* little (really a very little!) CPU usage.                          */
  setsockopt(can_st_, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

  if (bind(can_st_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0)
  {
    perror("bind");
    return false;
  }

  ROS_INFO_NAMED(TAG_, "End openCan.");
  return true;
}

int CanDriver::cmdMsg2CanFrame(const adam_msgs::VehicleCmdConstPtr &msg, struct canfd_frame *cf)
{
  memset(cf, 0, sizeof(struct canfd_frame));

  cf->can_id = t_can_addr_;
  cf->len = 8;

  if (msg->mode == 1)
  {
    // 前进
    if (!util::equal(msg->brake_cmd, 0., 0.01))
    {
      // 刹车
      cf->data[0] = 0x09;
      cf->data[3] = u_char(msg->brake_cmd);
    }
    else
    {
      cf->data[0] = 0x03;
      cf->data[3] = u_char(msg->accel_cmd);
    }
    // TODO: 转速、加速度、转角的转换
    ushort rpm = ushort(msg->ctrl_cmd.speed);
    cf->data[1] = (rpm | 0xFF);
    cf->data[2] = ((rpm << 8) | 0xFF);

    ushort steering = ushort(msg->ctrl_cmd.steering_angle);
    cf->data[5] = (steering | 0xFF);
    cf->data[6] = ((steering << 8) | 0xFF);
  }
  else if (msg->mode == 0)
  {
    // 倒车
    if (!(util::equal(msg->brake_cmd, 0., 0.01)))
    {
      cf->data[0] = 0x09;
      cf->data[3] = u_char(msg->brake_cmd);
    }
    else
    {
      cf->data[0] = 0x05;
      cf->data[3] = u_char(msg->accel_cmd);
    }
    // TODO: 转速、加速度、转角的转换
    ushort rpm = ushort(msg->ctrl_cmd.speed);
    cf->data[1] = (rpm | 0xFF);
    cf->data[2] = ((rpm << 8) | 0xFF);

    ushort steering = ushort(msg->ctrl_cmd.steering_angle);
    cf->data[5] = (steering | 0xFF);
    cf->data[6] = ((steering << 8) | 0xFF);
  }
  else
  {
    return CAN_MTU + 1;
  }

  return CAN_MTU;
}

void CanDriver::controlCmdCB(const adam_msgs::VehicleCmdConstPtr &msg)
{
  required_mtu_ = cmdMsg2CanFrame(msg, &frame_);

  // TODO: 暂时还没用上
  if (required_mtu_ > CAN_MTU)
  {
    ROS_ERROR_NAMED(TAG_, "Error in cmdMsg2CanFrame.");
    return;

    // /* check if the frame_ fits into the CAN netdevice */
    // if (ioctl(can_st_, SIOCGIFMTU, &ifr_) < 0)
    // {
    //   perror("SIOCGIFMTU");
    // }
    // mtu_ = ifr_.ifr_mtu;

    // if (mtu_ != CANFD_MTU)
    // {
    //   printf("CAN interface is not CAN FD capable - sorry.\n");
    // }

    // /* interface is ok - try to switch the socket into CAN FD mode */
    // if (setsockopt(can_st_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
    //                &enable_canfd_, sizeof(enable_canfd_)))
    // {
    //   printf("error when enabling CAN FD support\n");
    // }

    // /* ensure discrete CAN FD length values 0..8, 12, 16, 20, 24, 32, 64 */
    // frame_.len = can_dlc2len(can_len2dlc(frame_.len));
  }

  /* send frame */
  if (write(can_st_, &frame_, required_mtu_) != required_mtu_)
  {
    ROS_ERROR_NAMED(TAG_, "Error in write can packet.");
  }
  else
  {
    ROS_INFO_NAMED(TAG_, "Can Frame: id: %s, addr: %d, len: %d", ifr_.ifr_name, frame_.can_id, frame_.len);
  }
}
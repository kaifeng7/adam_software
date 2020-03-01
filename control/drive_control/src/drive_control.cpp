#include "drive_control/drive_control.h"

bool DriveControl::run()
{
  if (!init() || !openUDPPort())
  {
    ROS_ERROR("DriveControl: error.");
    return false;
  }

  ros::spin();
  return true;
}

bool DriveControl::init()
{
  ROS_INFO("DriveControl: Start init.");

  sub_cmd_ = nh_.subscribe<adam_msgs::VehicleCmd>("/vehicle_cmd", 1, boost::bind(&DriveControl::cmdCB, this, _1));
  sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, boost::bind(&DriveControl::odomCB, this, _1));
  pub_mode_stat_ = nh_.advertise<std_msgs::Int32>("/mode_stat", 1);
  pub_applied_ctrl_ = nh_.advertise<adam_msgs::ControlCommandStamped>("/applied_ctrl", 1);
  msg_applied_ctrl_.header.frame_id = "/base_link";

  pnh_.param<double>("pwm1_ml", param_pwm1_ml_, 15000.);
  pnh_.param<double>("pwm1_mh", param_pwm1_mh_, 15250.);
  pnh_.param<double>("pwm1_max", param_pwm1_max_, 15440.);
  pnh_.param<double>("pwm1_min", param_pwm1_min_, 15000.);
  pnh_.param<double>("pwm1_min_f", param_pwm1_min_f_, 15250.);
  pnh_.param<double>("pwm1_min_b", param_pwm1_min_b_, 14700.);
  pnh_.param<std::string>("pwm1_vf_string", param_pwm1_vf_string_, std::string("[]"));
  pnh_.param<std::string>("pwm1_vb_string", param_pwm1_vb_string_, std::string("[]"));
  pnh_.param<double>("pwm1_max_diff", param_pwm1_max_diff_, 40.);
  pnh_.param<double>("pwm2_ml", param_pwm2_ml_, 15600.);
  pnh_.param<double>("pwm2_mh", param_pwm2_mh_, 15700.);
  pnh_.param<double>("pwm2_max", param_pwm2_max_, 19250.);
  pnh_.param<double>("pwm2_min", param_pwm2_min_, 11450.);
  pnh_.param<double>("pwm2_max_diff", param_pwm2_max_diff_, 1000.);
  pnh_.param<std::string>("device_ip", param_device_ip_str_, "127.0.0.1");
  pnh_.param<int>("udp_port", param_udp_port_num_, 1024);
  pnh_.param<std::string>("mcu_ip", param_mcu_ip_str_, "192.168.1.101");
  pnh_.param<int>("mcu_udp_port", param_mcu_udp_port_, 1025);
  pnh_.param<double>("udp_wait", param_udp_wait_, 50.);
  pnh_.param<int>("pwm_duration", param_pwm_duration_, 60);
  pnh_.param<double>("default_acc", param_default_acc_, 1.0);
  pnh_.param<double>("default_ang_vel", param_default_ang_vel_, 0.175);
  pnh_.param<double>("v1", param_v1_, 1.0);
  pnh_.param<double>("v2", param_v2_, 2.0);
  pnh_.param<double>("v1_pwm", param_v1_pwm_, 15450.); // TODO 需要重新测量
  pnh_.param<double>("v2_pwm", param_v2_pwm_, 15480.);
  pnh_.param<double>("v3", param_v3_, -1.0);
  pnh_.param<double>("v4", param_v4_, -2.0);
  pnh_.param<double>("v3_pwm", param_v3_pwm_, 14700.);
  pnh_.param<double>("v4_pwm", param_v4_pwm_, 14500.);
  pnh_.param<double>("delta_ang_r", param_delta_ang_r_, 0.035);
  pnh_.param<double>("delta_ang_r_pwm", param_delta_ang_r_pwm_, 285.);
  pnh_.param<double>("delta_ang_l", param_delta_ang_l_, 0.035);
  pnh_.param<double>("delta_ang_l_pwm", param_delta_ang_l_pwm_, 315.);
  pnh_.param<bool>("human_drive", param_human_drive_, false);
  pnh_.param<bool>("use_speed_feedback", use_speed_feedback_, false);
  pnh_.param<double>("kp", Kp_, 0.);
  pnh_.param<double>("ki", Ki_, 0.);
  pnh_.param<double>("kd", Kd_, 0.);
  pnh_.param<double>("pwm1_stop", pwm1_stop_, 15000.);
  pnh_.param<double>("speed_feedback_duration", speed_feedback_duration_, 100);
  if (param_human_drive_)
    use_speed_feedback_ = false;
  else
    use_speed_feedback_ = true;
  udp_duration_.fromSec(param_udp_wait_ / 1000.);
  cur_mode_ = 1; // 初始化为前进
  pthread_mutex_init(&mutex_, NULL);

  if (!util::arrayParser(param_pwm1_vf_string_, pwm1_vf_) || pwm1_vf_.size() < 3)
  {
    ROS_ERROR("DriveControl: cannot parse pwm1_vf.");
    return false;
  }
  if (!util::arrayParser(param_pwm1_vb_string_, pwm1_vb_) || pwm1_vb_.size() < 3)
  {
    ROS_ERROR("DriveControl: cannot parse pwm1_vb.");
    return false;
  }
  std::reverse(pwm1_vb_.begin(), pwm1_vb_.end());

  if (!(param_v1_ < param_v2_ && param_v1_pwm_ < param_v2_pwm_ && param_v4_ < param_v3_ && param_v4_ < param_v3_pwm_))
  {
    ROS_ERROR("DriveControl: error in velocity's corresponding pwm.");
    return false;
  }

  cur_pwm1_ = (param_pwm1_ml_ + param_pwm1_mh_) / 2.0;
  cur_pwm2_ = (param_pwm2_ml_ + param_pwm2_mh_) / 2.0;
  cur_packet_.type = VAL_VEL;
  cur_packet_.syn = 0xFA;
  cur_packet_.dat.vel.liner[0] = cur_pwm1_;
  cur_packet_.dat.vel.angular[2] = cur_pwm2_;
  cur_packet_.syn_CR = param_pwm_duration_;
  cur_packet_.syn_LF = '\n';

  inet_aton(param_device_ip_str_.c_str(), &device_ip_);
  inet_aton(param_mcu_ip_str_.c_str(), &MCU_ip_);
  socket_fd_ = -1;
  ROS_INFO_STREAM("Opening UDP socket: address " << param_device_ip_str_);
  ROS_INFO_STREAM("Opening UDP socket: port " << param_udp_port_num_);

  ROS_INFO("DriveControl: End init.");

  return true;
}

/**
 * @brief 档位切换函数，前进档变为倒退档或倒退档变为前进挡，1s 左右完成换挡。注意：仅在车速降为 0 时才可以换挡，也就是调用此函数
 * 
 * @param target_mode 
 * @return true 
 * @return false 
 */
bool DriveControl::changeMode(int target_mode)
{
  if (target_mode == 0 || cur_mode_ == target_mode)
  {
    ROS_ERROR("DriveControl: error in changeMode");
    return false;
  }

  if (target_mode == 1)
  {
    int step_num = 1000 / param_udp_wait_; // 1s 内发送的包数量
    cur_packet_.dat.vel.liner[0] = param_pwm1_mh_;
    double step_size = (param_pwm1_max_ - param_pwm1_mh_) / step_num;
    for (int i = 0; i < step_num; i++)
    {
      cur_packet_.dat.vel.liner[0] += step_size;
      if (!sendPacket(socket_fd_, (uint8_t *)&cur_packet_, PACKET_SIZE, 0, (struct sockaddr *)&mcu_addr_, mcu_addr_len_))
      {
        return false;
      }
    }

    cur_packet_.dat.vel.liner[0] = (param_pwm1_ml_ + param_pwm1_mh_) / 2.0;
    if (!sendPacket(socket_fd_, (uint8_t *)&cur_packet_, PACKET_SIZE, 0, (struct sockaddr *)&mcu_addr_, mcu_addr_len_))
    {
      return false;
    }
  }
  else if (target_mode == -1)
  {
    int step_num = 1000 / param_udp_wait_; // 1s 内发送的包数量
    cur_packet_.dat.vel.liner[0] = param_pwm1_ml_;
    double step_size = (param_pwm1_ml_ - param_pwm1_min_) / step_num;
    for (int i = 0; i < step_num; i++)
    {
      cur_packet_.dat.vel.liner[0] -= step_size;
      if (!sendPacket(socket_fd_, (uint8_t *)&cur_packet_, PACKET_SIZE, 0, (struct sockaddr *)&mcu_addr_, mcu_addr_len_))
      {
        return false;
      }
    }

    cur_packet_.dat.vel.liner[0] = (param_pwm1_ml_ + param_pwm1_mh_) / 2.0;
    if (!sendPacket(socket_fd_, (uint8_t *)&cur_packet_, PACKET_SIZE, 0, (struct sockaddr *)&mcu_addr_, mcu_addr_len_))
    {
      return false;
    }
  }
  else
  {
    ROS_ERROR("DriveControl: error in changeMode");
    return false;
  }
  cur_mode_ = target_mode;
  return true;
}

bool DriveControl::stopCar()
{
  double pwm;
  if (cur_mode_ > 0)
  {
    // 前进状态时的刹车 pwm
    pwm = (param_pwm1_ml_ + param_pwm1_min_) / 2.0;
  }
  else if (cur_mode_ < 0)
  {
    pwm = (param_pwm1_mh_ + param_pwm1_max_) / 2.0;
  }
  else
  {
    ROS_ERROR("DriveControl: error mode.");
    return false;
  }

  int max_cnt = 50;
  while (!util::equal(cur_odom_.twist.twist.linear.x, 0, 0.1))
  {
    if (--max_cnt < 0)
    {
      break;
    }
    ROS_INFO("DriveControl: stoping car, cur_vel: %.4f, cur_pwm: %.4f", cur_odom_.twist.twist.linear.x, cur_packet_.dat.vel.liner[0]);
    if (!sendPwm(pwm, cur_pwm2_, param_pwm_duration_))
    {
      return false;
    }
  }
  return true;
}

void DriveControl::odomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
  cur_odom_ = *msg;
}

/**
 * @brief 根据控制消息发送相应 pwm 到 stm32
 * 
 * @param msg 
 */
void DriveControl::cmdCB(const adam_msgs::VehicleCmd::ConstPtr &msg)
{
  cur_cmd_ = *msg;
  nav_msgs::Odometry odom = cur_odom_;
  double pwm1 = cur_pwm1_, pwm2 = cur_pwm2_;
  double delta_pwm1, delta_pwm2;
  double target_pwm1, target_pwm2;

  if ((msg->header.stamp - odom.header.stamp).toSec() > 0.5)
  {
    if (!param_human_drive_)
    {
      ROS_WARN("DriveControl: longtime waiting for odom.");
      ROS_WARN("cmd time is %f, odom time is %f", msg->header.stamp, odom.header.stamp);

      return;
    }
  }

  // 档位不一致，直接换挡
  if (cur_mode_ != cur_cmd_.mode)
  {
    pthread_mutex_lock(&mutex_);
    if (!util::equal(odom.twist.twist.linear.x, 0, 0.1))
    {
      ROS_WARN("DriveControl: stop before change mode.");
      stopCar();
      cur_pwm1_ = (param_pwm1_ml_ + param_pwm2_mh_) / 2.;
    }
    ROS_INFO("DriveControl: change mode.");
    if (changeMode(cur_cmd_.mode))
    {
      msg_mode_stat_.data = cur_mode_;
      pub_mode_stat_.publish(msg_mode_stat_);
    }
    else
    {
      ROS_ERROR("DriveControl: cannot change mode.");
    }
    pthread_mutex_unlock(&mutex_);
    return;
  }

  double acc = 0.;
  if (cur_mode_ > 0)
  {
    if(odom.twist.twist.linear.x < -0.0001){
      ROS_ERROR("DriveControl: hall_speed < 0");
      pwm1 = pwm1_stop_;
      return;
    }
    if (msg->ctrl_cmd.speed < -0.0001)
    {
      ROS_ERROR("DriveControl: cur mode > 0 but cmd.speed < 0.");
      return;
    }
    if (util::equal(odom.twist.twist.linear.x, msg->ctrl_cmd.speed, 0.18))
    {
      // 当前车速与目标速度近似，不再加减速
      err_[2] = err_[1];
      err_[1] = err_[0];
      // 这里误差实时计算用于下次速度差异较大时的控制
      err_[0] = msg->ctrl_cmd.speed - odom.twist.twist.linear.x;
      acc = 0.;
    }
    else
    {
      err_[2] = err_[1];
      err_[1] = err_[0];
      err_[0] = msg->ctrl_cmd.speed - odom.twist.twist.linear.x;
      if (!use_speed_feedback_)
      {
        // 通过查速度-pwm 关系表(pwm1_vf_)以及插值计算目标速度对应的 pwm
        auto lower = std::lower_bound(pwm1_vf_.begin(), pwm1_vf_.end(), msg->ctrl_cmd.speed, [](geometry_msgs::Point32 it, double v) -> bool { return it.y < v; });
        lower = lower == pwm1_vf_.end() ? pwm1_vf_.end() - 1 : lower;
        auto higher = (lower - 1) < pwm1_vf_.begin() ? pwm1_vf_.begin() : (lower - 1);
        target_pwm1 = lower->x + (higher->x - lower->x) * (msg->ctrl_cmd.speed - lower->y + 0.001) / (higher->y - lower->y + 0.001);
    printf("target_pwm1%lf\n", target_pwm1);
        if (pwm1 > target_pwm1)
        {
          // 减速前进
          acc = (util::equal(msg->brake_cmd, 0, 0.001) ? -param_default_acc_ : -msg->brake_cmd);
        }
        else
        {
          // 加速前进
          acc = (util::equal(msg->accel_cmd, 0, 0.001) ? param_default_acc_ : msg->accel_cmd);
        }
      }
      else
      {
        ROS_INFO("use speed feedback");
        //使用速度闭环反馈
        acc = Kp_ * (err_[0] - err_[1]) + Ki_ * (err_[0]) + Kd_ * ((err_[0] - err_[1]) - (err_[1] - err_[2]));
	      printf("err[0]=%.2f,err[1]=%.2f,err[2]=%.2f\n",err_[0],err_[1],err_[2]);
        if (acc > 3.0)
          acc = 3.0;
        else if (acc < -0.5)
          acc = -0.5;
      }
      delta_pwm1 = (param_v2_pwm_ - param_v1_pwm_) / (param_v2_ - param_v1_) * acc;
      printf("param_v2_pwm_=%f, param_v1_pwm_=%f, param_v2_=%f, param_v1_=%f\n", param_v2_pwm_, param_v1_pwm_, param_v2_, param_v1_);
      printf("acc=%f, delta_pwm1=%f\n", acc, delta_pwm1);
    }
    if(util::equal(msg->ctrl_cmd.speed, 0, 0.001)){
      // 控制指令为0时不允许有0.18误差，直接置为stop时对应的频率
      pwm1 = pwm1_stop_;
      delta_pwm1 = 0.;
    }
      
    
  }
  else if (cur_mode_ < 0)
  {
    if (msg->ctrl_cmd.speed > 0.0001)
    {
      ROS_ERROR("DriveControl: cur mode <0 but cmd.speed >I 0.");
      return;
    }
    if (util::equal(odom.twist.twist.linear.x, msg->ctrl_cmd.speed, 0.2))
    {
      // 当前车速与目标速度近似，不再加减速
      acc = 0.;
    }
    else
    {
      auto lower = std::lower_bound(pwm1_vb_.begin(), pwm1_vb_.end(), msg->ctrl_cmd.speed, [](geometry_msgs::Point32 it, double v) -> bool { return it.y < v; });
      lower = lower == pwm1_vb_.end() ? pwm1_vb_.end() - 1 : lower;
      auto higher = (lower - 1) < pwm1_vb_.begin() ? pwm1_vb_.begin() : (lower - 1);
      target_pwm1 = higher->x + (lower->x - higher->x) * (higher->y - msg->ctrl_cmd.speed + 0.001) / (higher->y - lower->y + 0.001);
      if (odom.twist.twist.linear.x > msg->ctrl_cmd.speed)
      {
        // 加速倒退
        acc = (util::equal(msg->accel_cmd, 0, 0.001) ? -param_default_acc_ : -msg->accel_cmd);
      }
      else
      {
        // 减速倒退
        acc = (util::equal(msg->brake_cmd, 0, 0.001) ? param_default_acc_ : msg->brake_cmd);
      }
      delta_pwm1 = (param_v3_pwm_ - param_v4_pwm_) / (param_v3_ - param_v4_) * acc;
    }
  }
  else
  {
    // TODO: do something?
    ROS_ERROR("DriveControl: error.");
  }

  if (!use_speed_feedback_)
  {
    if (util::equal(pwm1, target_pwm1, 1))
    {
      pwm1 = target_pwm1;
    }
    else
    {
      pwm1 += delta_pwm1 * param_udp_wait_ / 1000.;
    }
  }
  else
  {
    pwm1 += delta_pwm1 * param_udp_wait_ / 1000.;
    //限制
    if (pwm1 < 14600)
      pwm1 = 14600;
    if (pwm1 > 15500)
      pwm1 = 15500;
  }

  if (cur_mode_ > 0)
  {
    if (!util::equal(msg->ctrl_cmd.speed, 0., 0.1))
    {
      pwm1 = util::cut(pwm1, param_pwm1_min_f_, param_pwm1_max_);
    }
    pwm1 = util::cut(pwm1, (param_pwm1_min_ + param_pwm1_ml_) / 2., param_pwm1_max_);
    auto lower = std::lower_bound(pwm1_vf_.begin(), pwm1_vf_.end(), pwm1, [](geometry_msgs::Point32 it, double pwm) -> bool { return it.x < pwm; });
    lower = lower == pwm1_vf_.end() ? pwm1_vf_.end() - 1 : lower;
    auto higher = (lower - 1) < pwm1_vf_.begin() ? pwm1_vf_.begin() : (lower - 1);
    msg_applied_ctrl_.cmd.speed = lower->y + (higher->y - lower->y) * (pwm1 - lower->x + 0.001) / (higher->x - lower->x);
  }
  else
  {
    if (!util::equal(msg->ctrl_cmd.speed, 0., 0.1))
    {
      pwm1 = util::cut(pwm1, param_pwm1_min_, param_pwm1_min_b_);
    }
    pwm1 = util::cut(pwm1, param_pwm1_min_, (param_pwm1_mh_ + param_pwm1_max_) / 2.);
    auto lower = std::lower_bound(pwm1_vb_.begin(), pwm1_vb_.end(), pwm1, [](geometry_msgs::Point32 it, double pwm) -> bool { return it.x < pwm; });
    lower = lower == pwm1_vb_.end() ? pwm1_vb_.end() - 1 : lower;
    auto higher = (lower - 1) < pwm1_vb_.begin() ? pwm1_vb_.begin() : (lower - 1);
    msg_applied_ctrl_.cmd.speed = higher->y + (lower->y - higher->y) * (higher->x - pwm1 + 0.001) / (higher->x - lower->x);
  }
  msg_applied_ctrl_.cmd.acceleration = acc;

  // 根据 pwm 近似计算当前转角
  double cur_ang = 0., ang_vel = msg->steer_cmd;
  if (cur_pwm2_ > param_pwm2_mh_)
  {
    cur_ang += (cur_pwm2_ - param_pwm2_mh_) / param_delta_ang_r_pwm_ * param_delta_ang_r_;
  }
  else if (cur_pwm2_ < param_pwm2_ml_)
  {
    cur_ang -= (param_pwm2_ml_ - cur_pwm2_) / param_delta_ang_l_pwm_ * param_delta_ang_l_;
  }

  // 计算转角速度对应的 pwm 变化率: delta_pwm2_
  if (util::equal(msg->ctrl_cmd.steering_angle, cur_ang, 0.001))
  {
    // 当前转角与目标转角近似
    delta_pwm2_ = 0.;
  }
  else if (cur_ang > msg->ctrl_cmd.steering_angle)
  {
    if (ang_vel > 0.001)
    {
      // error 当前转角大于控制命令的转角时，无法再右转
      ROS_ERROR("DriveControl: cur_ang > cmd.steering_angle but ang_vel > 0.");
      delta_pwm2_ = 0.;
    }
    else
    {
      // 左转
      if (util::equal(ang_vel, 0., 0.001))
      {
        // 未指定转角速度，使用默认转角速度
        ang_vel = -param_default_ang_vel_;
      }
      delta_pwm2_ = ang_vel / param_delta_ang_l_ * param_delta_ang_l_pwm_;
    }
  }
  else if (cur_ang < msg->ctrl_cmd.steering_angle)
  {
    if (ang_vel < -0.001)
    {
      // error 当前转角小于控制命令的转角时，无法再左转
      ROS_ERROR("DriveControl: cur_ang < cmd.steering_angle but ang_vel < 0.");
      delta_pwm2_ = 0.;
    }
    else
    {
      // 右转
      if (util::equal(ang_vel, 0., 0.001))
      {
        // 未指定转角速度，使用默认转角速度
        ang_vel = param_default_ang_vel_;
      }
      delta_pwm2_ = ang_vel / param_delta_ang_r_ * param_delta_ang_r_pwm_;
    }
  }

  pwm2 += delta_pwm2_ * param_udp_wait_ / 1000.;
  if (pwm2 > param_pwm2_mh_)
  {
    msg_applied_ctrl_.cmd.steering_angle = (pwm2 - param_pwm2_mh_) / param_delta_ang_r_pwm_ * param_delta_ang_r_;
  }
  else if (pwm2 < param_pwm2_ml_)
  {
    msg_applied_ctrl_.cmd.steering_angle = -(param_pwm2_ml_ - pwm2) / param_delta_ang_l_pwm_ * param_delta_ang_l_;
  }
  msg_applied_ctrl_.cmd.steering_angle_velocity = ang_vel;

  // 发送对应 pwm 到底盘
  ROS_INFO("DriveControl: sending pwm. pwm1: %.2f, pwm2: %.2f, target speed: %.2f, cur speed: %.2f, target angle: %.2f", pwm1, pwm2, msg->ctrl_cmd.speed, odom.twist.twist.linear.x, msg->ctrl_cmd.steering_angle);
  sendPwm(pwm1, pwm2, param_pwm_duration_);
  msg_applied_ctrl_.header.stamp = msg->header.stamp;
  pub_applied_ctrl_.publish(msg_applied_ctrl_);
}

bool DriveControl::openUDPPort()
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

bool DriveControl::sendPacket(int fd, const void *buf, size_t n,
                              int flags, __CONST_SOCKADDR_ARG addr,
                              socklen_t addr_len)
{
  ssize_t nbytes = sendto(fd, buf, n, flags, addr, addr_len);
  if ((size_t)nbytes != n)
  {
    ROS_ERROR("cannot send cmd_vel to MCU!");
    return false;
  }

  ros::spinOnce();
  return true;
}

bool DriveControl::sendPwm(double pwm1, double pwm2, double duration)
{
  if (cur_pwm1_ - param_pwm1_max_diff_ > pwm1)
  {
    pwm1 = cur_pwm1_ - param_pwm1_max_diff_;
  }
  else if (cur_pwm1_ + param_pwm1_max_diff_ < pwm1)
  {
    pwm1 = cur_pwm1_ + param_pwm1_max_diff_;
  }
  if (cur_pwm2_ - param_pwm2_max_diff_ > pwm2)
  {
    pwm2 = cur_pwm2_ - param_pwm2_max_diff_;
  }
  else if (cur_pwm2_ + param_pwm2_max_diff_ < pwm2)
  {
    pwm2 = cur_pwm2_ + param_pwm2_max_diff_;
  }
  cur_pwm1_ = util::cut(pwm1, param_pwm1_min_, param_pwm1_max_);
  cur_pwm2_ = util::cut(pwm2, param_pwm2_min_, param_pwm2_max_);
  cur_packet_.dat.vel.liner[0] = cur_pwm1_;
  cur_packet_.dat.vel.angular[2] = cur_pwm2_;
  cur_packet_.syn_CR = duration;
  return sendPacket(socket_fd_, (uint8_t *)&cur_packet_, PACKET_SIZE, 0, (struct sockaddr *)&mcu_addr_, mcu_addr_len_);
}

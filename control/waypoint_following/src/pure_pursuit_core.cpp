/*
 * @Author: fengk 
 * @Date: 2019-04-15 22:18:08 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-17 17:57:27
 */

#include "pure_pursuit.h"

// Constructor
PurePursuit::PurePursuit() : pnh_("~"),
                             RADIUS_MAX(9e10),
                             KAPPA_MIN(1 / RADIUS_MAX),
                             bPath(false),
                             bCurrentPose(false),
                             bCarBasicInfo(false),
                             bCarLimitInfo(false),
                             bCmdVel(false)
{
  initForROS();
}

// Destructor
PurePursuit::~PurePursuit()
{
}

void PurePursuit::initForROS()
{
  // ros parameter settings
  pnh_.param("is_linear_interpolation", is_linear_interpolation, false); //是否进行线性插值
  pnh_.param("lookahead_distance_ratio", lookahead_distance_ratio_, 2.0);
  pnh_.param("minimum_lookahead_distance", minimum_lookahead_distance_, 6.0);
  pnh_.param("loop_rate", LOOP_RATE_, 20);

  // setup subscriber
  sub_Path = nh_.subscribe("/sub_path", 10, &PurePursuit::callbackFromPath, this);                    //订阅路径规划出来的路径点
  sub_CurrentPose = nh_.subscribe("/sub_pose", 10, &PurePursuit::callbackFromCurrentPose, this);      //订阅机器人发布的姿态
  sub_CommandVelocity = nh_.subscribe("/sub_velocity", 10, &PurePursuit::callbackFromVelocity, this); //订阅机器人发布的姿态
  sub_CarBasicInfo = nh_.subscribe("/car_basic_info", 1, &PurePursuit::callbackGetCarBasicInfo, this);

  sub_CarLimitInfo = nh_.subscribe("/car_limit_info", 1, &PurePursuit::callbackGetCarLimitInfo, this); // setup publisher
  pub_VehicleCmd = nh_.advertise<adam_msgs::VehicleCmd>("/cmd_vel", 10);                               //发布机器人运动命令

  pub_NextWaypoint = pnh_.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
  pub_NextTargetPosition = pnh_.advertise<visualization_msgs::Marker>("next_target_mark", 0);
  pub_SearchRadius = pnh_.advertise<visualization_msgs::Marker>("search_circle_mark", 0);
  //pub14_ = pnh_.advertise<visualization_msgs::Marker>("line_point_mark", 0); // debug tool
  pub_TrajectoryCircle = pnh_.advertise<visualization_msgs::Marker>("trajectory_circle_mark", 0);
  pub_AngularGravity = pnh_.advertise<std_msgs::Float32>("angular_gravity", 0);
  pub_DeviationCurrentPosition = pnh_.advertise<std_msgs::Float32>("deviation_of_current_position", 0);
  // pub7_ = nh.advertise<std_msgs::Bool>("wf_stat", 0);
}

void PurePursuit::MainLoop()
{
  ROS_INFO_STREAM("pure pursuit node start");
  ros::Rate loop_rate(LOOP_RATE_);
  while (ros::ok())
  {
    ros::spinOnce();
    //当订阅话题都收到数据时，开始pure_pursuit算法
    if (!bCurrentPose)
    {
      ROS_WARN("Current Pose topic is not subscribed yet ... ");
      loop_rate.sleep();
      continue;
    }
    else if (!bPath)
    {
      ROS_WARN("Path topic is not subscribed yet ... ");
      loop_rate.sleep();
      continue;
    }
    else if (!bCarBasicInfo || !bCarLimitInfo)
    {
      ROS_WARN("Car Info topic is not subscribed yet ... ");
      loop_rate.sleep();
      continue;
    }

    m_LookaheadDistance = LookaheadDistance(); //计算机器人的预瞄距离，主要和无人车当前的速度，转向角度有关
    double kappa = 0;
    int i_next_waypoint;
    bool is_get_curvature = IsGetCurvature(i_next_waypoint, kappa); //计算机器人的曲率
    ROS_INFO_STREAM("curvature:" << is_get_curvature);
    publishTwist(is_get_curvature, kappa);

    // for visualization with Rviz
    pub_NextWaypoint.publish(displayNextWaypoint(m_Path.poses.at(i_next_waypoint).pose.position));
    pub_SearchRadius.publish(displaySearchRadius(m_CurrentPose.position, m_LookaheadDistance));
    pub_NextTargetPosition.publish(displayNextTarget(m_NextTargetPosition));
    pub_TrajectoryCircle.publish(displayTrajectoryCircle(generateTrajectoryCircle(m_NextTargetPosition, m_CurrentPose)));
    std_msgs::Float32 angular_gravity_msg;
    angular_gravity_msg.data = computeAngularGravity(m_CommandVelocity, kappa);
    pub_AngularGravity.publish(angular_gravity_msg);

    publishDeviationCurrentPosition(m_CurrentPose.position, m_Path);

    //bCurrentPose = false;
    loop_rate.sleep();
  }
}

void PurePursuit::publishTwist(const bool &is_get_curvature, const double &kappa)
{
  adam_msgs::VehicleCmd vc;
  vc.header.stamp = ros::Time::now();
  vc.ctrl_cmd.speed = is_get_curvature ? m_CommandVelocity : 0;
  vc.ctrl_cmd.steering_angle = is_get_curvature ? atan(m_CarBasicInfo.wheel_base * kappa) * vc.ctrl_cmd.speed : 0;
  if (vc.ctrl_cmd.speed >= 0)
    vc.mode = 1;
  else
    vc.mode = 0;
  pub_VehicleCmd.publish(vc);
}

double PurePursuit::LookaheadDistance()
{ //最大预瞄距离等于当前速度的ratio倍

  double maximum_lookahead_distance = m_CurrentVelocity * 10; //最大预瞄距离为当前速度的10倍
  double ld = m_CurrentVelocity * lookahead_distance_ratio_;  //设置预瞄距离，线性比率为2

  return ld < minimum_lookahead_distance_ ? minimum_lookahead_distance_ : ld > maximum_lookahead_distance ? maximum_lookahead_distance : ld;
}

double PurePursuit::computeAngularGravity(double velocity, double kappa)
{
  const double gravity = 9.80665;
  return (velocity * velocity) / (1.0 / kappa * gravity);
}

void PurePursuit::publishDeviationCurrentPosition(const geometry_msgs::Point &point,
                                                  const nav_msgs::Path &path)
{
  // Calculate the deviation of current position from the waypoint approximate line

  if (path.poses.size() < 3)
  {
    return;
  }

  double a, b, c;
  utility.getLinearEquation(path.poses.at(2).pose.position, path.poses.at(1).pose.position, a, b, c);

  std_msgs::Float32 msg;
  msg.data = utility.getDistanceBetweenLineAndPoint(point, a, b, c);

  pub_DeviationCurrentPosition.publish(msg);
}

void PurePursuit::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
  m_CurrentPose = msg->pose;
  bCurrentPose = true;
}

void PurePursuit::callbackFromPath(const nav_msgs::PathConstPtr &msg)
{
  if (msg->poses.empty())
    m_CurrentVelocity = 0;

  m_Path = *msg;
  bPath = true;
}
void PurePursuit::callbackFromVelocity(const geometry_msgs::TwistConstPtr &msg)
{
  static double pre_time = ros::Time::now().toSec();
  static double pre_vel = msg->linear.x;
  double cur_time = ros::Time::now().toSec();
  double cur_vel = msg->linear.x;
  double diff_time = cur_time - pre_time;
  if (diff_time > 0.0001)
  {
    double acc = (cur_vel - pre_vel) / diff_time;
    if (std::abs(acc) > 1.0)
    {
      cur_vel = pre_vel + acc * diff_time;
    }
  }
  else
  {
    ROS_ERROR("error in callbackFromVelocity");
  }
  m_CurrentVelocity = cur_vel;
  m_CommandVelocity = cur_vel;
  bCmdVel = true;
  pre_time = cur_time;
  pre_vel = cur_vel;
}

// 计算机器人的曲率半径
double PurePursuit::calcCurvature(const geometry_msgs::Point &target)
{
  double kappa;
  double denominator = pow(utility.getPlaneDistance(target, m_CurrentPose.position), 2); //分母
  double numerator = 2 * utility.calcRelativeCoordinate(target, m_CurrentPose).y;        //分子

  if (denominator != 0)
    kappa = numerator / denominator;
  else
  {
    if (numerator > 0)
      kappa = KAPPA_MIN; // KAPPA_MIN_ = 1/RADIUS_MAX_(9e10)
    else
      kappa = -KAPPA_MIN;
  }
  return kappa;
}
void PurePursuit::callbackGetCarBasicInfo(const adam_msgs::CarBasicInfoConstPtr &msg)
{
  m_CarBasicInfo = *msg;
  bCarBasicInfo = true;
}

void PurePursuit::callbackGetCarLimitInfo(const adam_msgs::CarLimitInfoConstPtr &msg)
{
  m_CarLimitInfo = *msg;
  bCarLimitInfo = true;
}

// 线性差值
bool PurePursuit::interpolateNextTarget(int next_waypoint_index, geometry_msgs::Point *next_target)
{
  const double ERROR = pow(10, -5); // 0.00001

  int path_size = m_Path.poses.size();
  if (next_waypoint_index == path_size - 1)
  {
    *next_target = m_Path.poses.at(next_waypoint_index).pose.position;
    return true;
  }
  double search_radius = m_LookaheadDistance;
  geometry_msgs::Point zero_p;
  geometry_msgs::Point end = m_Path.poses.at(next_waypoint_index).pose.position;
  geometry_msgs::Point start = m_Path.poses.at(next_waypoint_index - 1).pose.position;

  // let the linear equation be "ax + by + c = 0"
  // if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
  double a = 0;
  double b = 0;
  double c = 0;
  double get_linear_flag = utility.getLinearEquation(start, end, a, b, c);
  if (!get_linear_flag)
    return false;

  // let the center of circle be "(x0,y0)", in my code , the center of circle is vehicle position
  // the distance  "d" between the foot of a perpendicular line and the center of circle is ...
  //    | a * x0 + b * y0 + c |
  // d = -------------------------------
  //          √( a~2 + b~2)
  double d = utility.getDistanceBetweenLineAndPoint(m_CurrentPose.position, a, b, c);

  // ROS_INFO("a : %lf ", a);
  // ROS_INFO("b : %lf ", b);
  // ROS_INFO("c : %lf ", c);
  // ROS_INFO("distance : %lf ", d);

  if (d > search_radius) //主要判断机器人规划的路径是否在机器人的预瞄距离内，如果不在，则报错
    return false;

  // unit vector of point 'start' to point 'end'
  tf::Vector3 v((end.x - start.x), (end.y - start.y), 0);
  tf::Vector3 unit_v = v.normalize();

  // normal unit vectors of v
  tf::Vector3 unit_w1 = utility.rotateUnitVector(unit_v, 90);  // rotate to counter clockwise 90 degree
  tf::Vector3 unit_w2 = utility.rotateUnitVector(unit_v, -90); // rotate to counter clockwise 90 degree

  // the foot of a perpendicular line
  geometry_msgs::Point h1;
  h1.x = m_CurrentPose.position.x + d * unit_w1.getX();
  h1.y = m_CurrentPose.position.y + d * unit_w1.getY();
  h1.z = m_CurrentPose.position.z;

  geometry_msgs::Point h2;
  h2.x = m_CurrentPose.position.x + d * unit_w2.getX();
  h2.y = m_CurrentPose.position.y + d * unit_w2.getY();
  h2.z = m_CurrentPose.position.z;

  // ROS_INFO("error : %lf", error);
  // ROS_INFO("whether h1 on line : %lf", h1.y - (slope * h1.x + intercept));
  // ROS_INFO("whether h2 on line : %lf", h2.y - (slope * h2.x + intercept));

  // check which of two foot of a perpendicular line is on the line equation
  geometry_msgs::Point h;
  if (fabs(a * h1.x + b * h1.y + c) < ERROR)
  {
    h = h1;
    //   ROS_INFO("use h1");
  }
  else if (fabs(a * h2.x + b * h2.y + c) < ERROR)
  {
    //   ROS_INFO("use h2");
    h = h2;
  }
  else
  {
    return false;
  }

  // get intersection[s]
  // if there is a intersection
  if (d == search_radius)
  {
    *next_target = h;
    return true;
  }
  else
  {
    // if there are two intersection
    // get intersection in front of vehicle
    double s = sqrt(pow(search_radius, 2) - pow(d, 2));
    geometry_msgs::Point target1;
    target1.x = h.x + s * unit_v.getX();
    target1.y = h.y + s * unit_v.getY();
    target1.z = m_CurrentPose.position.z;

    geometry_msgs::Point target2;
    target2.x = h.x - s * unit_v.getX();
    target2.y = h.y - s * unit_v.getY();
    target2.z = m_CurrentPose.position.z;

    // ROS_INFO("target1 : ( %lf , %lf , %lf)", target1.x, target1.y, target1.z);
    // ROS_INFO("target2 : ( %lf , %lf , %lf)", target2.x, target2.y, target2.z);
    // displayLinePoint(a, b, c, target1, target2, h);  // debug tool

    // check intersection is between end and start
    double interval = utility.getPlaneDistance(end, start);
    if (utility.getPlaneDistance(target1, end) < interval)
    {
      // ROS_INFO("result : target1");
      *next_target = target1;
      return true;
    }
    else if (utility.getPlaneDistance(target2, end) < interval)
    {
      // ROS_INFO("result : target2");
      *next_target = target2;
      return true;
    }
    else
    {
      // ROS_INFO("result : false ");
      return false;
    }
  }
}
//获取将要走的目标点，只有当目标点与机器人当前位置之间的距离大于预瞄距离才有效（最后一个waypoint除外），返回i_NextWaypoint
int PurePursuit::getNextWaypoint()
{
  int path_size = m_Path.poses.size();

  if (path_size == 0) //如果没有收到路径
  {
    return -1;
  }

  // look for the next waypoint.
  for (int i = 0; i < path_size; i++)
  {
    // if search waypoint is the last
    if (i == (path_size - 1)) //如果最近点为最后一个点
    {
      ROS_INFO_STREAM("search waypoint is the last:" << i);
      return i;
    }

    // if there exists an effective waypoint
    if (utility.getPlaneDistance(m_Path.poses.at(i).pose.position, m_CurrentPose.position) > m_LookaheadDistance)
    {
      return i;
    }
  }

  // if this program reaches here , it means we lost the waypoint!
  return -1;
}

bool PurePursuit::IsGetCurvature(int &i_next_waypoint, double &output_kappa)
{
  // search next waypoint
  i_next_waypoint = getNextWaypoint(); //获取到距离当前位置最近的点
  if (i_next_waypoint == -1)
  {
    ROS_INFO("lost next waypoint");
    return false;
  }
  // check whether curvature is valid or not
  bool is_valid_curve = false;
  int path_size = utility.getSize(m_Path);
  for (int i = 0; i < path_size; i++)
  {
    if (utility.getPlaneDistance(m_Path.poses.at(i).pose.position, m_CurrentPose.position) > minimum_lookahead_distance_)
    {
      is_valid_curve = true;
      break;
    }
  }
  //why
  // if (!is_valid_curve)
  // {
  //   return false;
  // }
  // if is_linear_interpolation_ is false or next waypoint is first or last
  if (!is_linear_interpolation || i_next_waypoint == 0 || i_next_waypoint == (path_size - 1))
  {
    m_NextTargetPosition = m_Path.poses.at(i_next_waypoint).pose.position;
    output_kappa = calcCurvature(m_NextTargetPosition);
    return true;
  }

  // linear interpolation and calculate angular velocity
  bool interpolation = interpolateNextTarget(i_next_waypoint, &m_NextTargetPosition);

  if (!interpolation)
  {
    ROS_INFO_STREAM("lost target! ");
    return false;
  }

  // ROS_INFO("next_target : ( %lf , %lf , %lf)", next_target.x, next_target.y,next_target.z);

  output_kappa = calcCurvature(m_NextTargetPosition);
  return true;
}

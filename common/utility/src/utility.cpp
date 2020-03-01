/*
 * @Author: fengk 
 * @Date: 2019-04-12 15:43:39 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-16 22:20:30
 */
#include "utility.h"


double Utility::PointLength(geometry_msgs::Point p)
{
  return sqrt(pow(p.x, 2) + pow(p.y, 2));
}

double Utility::AngleBetweenTwoPoints(geometry_msgs::Point from, geometry_msgs::Point to)
{
  return atan2(to.y - from.y, to.x - from.x);
}

double Utility::FixNegativeAngle(const double &a)
{
  double angle = 0;
  if (a < -2.0 * M_PI || a > 2.0 * M_PI)
  {
    angle = fmod(a, 2.0 * M_PI);
  }
  else
    angle = a;

  if (angle < 0)
  {
    angle = 2.0 * M_PI + angle;
  }

  return angle;
}

double Utility::AngleBetweenTwoAnglesPositive(const double &a1, const double &a2)
{
  double diff = a1 - a2;
  if (diff < 0)
    diff = a2 - a1;

  if (diff > M_PI)
    diff = 2.0 * M_PI - diff;

  return diff;
}

int Utility::getSize(const nav_msgs::Path &path)
{
  if (path.poses.empty())
    return 0;
  else
    return path.poses.size();
}

bool Utility::isEmpty(const nav_msgs::Path &path)
{
  return path.poses.empty();
};

double Utility::getInterval(const nav_msgs::Path &path)
{
  if (path.poses.empty())
    return 0;

  // interval between 2 waypoints
  tf::Vector3 v1(path.poses[0].pose.position.x,
                 path.poses[0].pose.position.y, 0);

  tf::Vector3 v2(path.poses[1].pose.position.x,
                 path.poses[1].pose.position.y, 0);
  return tf::tfDistance(v1, v2);
}

geometry_msgs::Point Utility::getWaypointPosition(const nav_msgs::Path &path,const int &i_waypoint)
{
  geometry_msgs::Point p;
  if (i_waypoint > getSize(path) - 1 || i_waypoint < 0)
    return p;

  p = path.poses[i_waypoint].pose.position;
  return p;
}

geometry_msgs::Quaternion Utility::getWaypointOrientation(const nav_msgs::Path &path,const int &i_waypoint)
{
  geometry_msgs::Quaternion q;
  if (i_waypoint > getSize(path) - 1 || i_waypoint < 0)
    return q;

  q = path.poses[i_waypoint].pose.orientation;
  return q;
}

geometry_msgs::Pose Utility::getWaypointPose(const nav_msgs::Path &path,const int &i_waypoint)
{
  geometry_msgs::Pose pose;
  if (i_waypoint > getSize(path) - 1 || i_waypoint < 0)
    return pose;

  pose = path.poses[i_waypoint].pose;
  return pose;
}

bool Utility::isFront(const nav_msgs::Path &path,const int &i_waypoint,const geometry_msgs::Pose &current_pose) 
{
  double x = calcRelativeCoordinate(path.poses[i_waypoint].pose.position, current_pose).x;
  if (x < 0)
    return false;
  else
    return true;
}
double Utility::Deceleration(double distance,double prev_velocity)
{
  double deceleration;
  deceleration = prev_velocity*prev_velocity/(2*distance);
}
double Utility::DecelerateVelocity(double distance, double prev_velocity)
{
  double decel_ms = 1.0; // m/s
  double decel_velocity_ms = sqrt(2 * decel_ms * distance);

  std::cout << "velocity/prev_velocity :" << decel_velocity_ms << "/" << prev_velocity << std::endl;
  if (decel_velocity_ms < prev_velocity)
  {
    return decel_velocity_ms;
  }
  else
  {
    return prev_velocity;
  }
}

// calculation relative coordinate of point from current_pose frame
geometry_msgs::Point Utility::calcRelativeCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose)
{
  tf::Transform inverse;
  tf::poseMsgToTF(current_pose, inverse);
  tf::Transform transform = inverse.inverse();

  tf::Point p;
  pointMsgToTF(point_msg, p);
  tf::Point tf_p = transform * p;
  geometry_msgs::Point tf_point_msg;
  pointTFToMsg(tf_p, tf_point_msg);

  return tf_point_msg;
}

// calculation absolute coordinate of point on current_pose frame
geometry_msgs::Point Utility::calcAbsoluteCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose)
{
  tf::Transform inverse;
  tf::poseMsgToTF(current_pose, inverse);

  tf::Point p;
  pointMsgToTF(point_msg, p);
  tf::Point tf_p = inverse * p;
  geometry_msgs::Point tf_point_msg;
  pointTFToMsg(tf_p, tf_point_msg);
  return tf_point_msg;
}

// distance between target 1 and target2 in 2-D
double Utility::getPlaneDistance(const geometry_msgs::Point &target1, const geometry_msgs::Point &target2)
{
  tf::Vector3 v1 = point2vector(target1);
  v1.setZ(0);
  tf::Vector3 v2 = point2vector(target2);
  v2.setZ(0);
  return tf::tfDistance(v1, v2);
}

double Utility::getPlaneDistance2(const geometry_msgs::Point &target1,const geometry_msgs::Point &target2)
{
  tf::Vector3 v1 = point2vector(target1);
  v1.setZ(0);
  tf::Vector3 v2 = point2vector(target2);
  v2.setZ(0);
  return tf::tfDistance2(v1, v2);
}

double Utility::getRelativeAngle(const geometry_msgs::Pose &waypoint_pose,const geometry_msgs::Pose &vehicle_pose)
{
  geometry_msgs::Point relative_p1 = calcRelativeCoordinate(waypoint_pose.position, vehicle_pose);
  geometry_msgs::Point p2;
  p2.x = 1.0;
  geometry_msgs::Point relative_p2 = calcRelativeCoordinate(calcAbsoluteCoordinate(p2, waypoint_pose), vehicle_pose);
  tf::Vector3 relative_waypoint_v(relative_p2.x - relative_p1.x, relative_p2.y - relative_p1.y,
                                  relative_p2.z - relative_p1.z);
  relative_waypoint_v.normalize();
  tf::Vector3 relative_pose_v(1, 0, 0);
  double angle = relative_pose_v.angle(relative_waypoint_v) * 180 / M_PI;
  // ROS_INFO("angle : %lf",angle);

  return angle;
}

// get closest waypoint from current pose
int Utility::getClosestWaypoint(const nav_msgs::Path &path,const geometry_msgs::Pose &current_pose)
{

  if (isEmpty(path))
    return -1;

  // search closest candidate within a certain meter
  double search_distance = 5.0;
  std::vector<int> waypoint_candidates;
  for (int i = 1; i < getSize(path); i++)
  {
    if (getPlaneDistance(getWaypointPosition(path,i), current_pose.position) > search_distance)
      continue;

    if (!isFront(path,i, current_pose))
      continue;

    double angle_threshold = 90;
    if (getRelativeAngle(getWaypointPose(path,i), current_pose) > angle_threshold)
      continue;

    waypoint_candidates.push_back(i);
  }

  // get closest waypoint from candidates
  if (!waypoint_candidates.empty())
  {
    int waypoint_min = -1;
    double distance_min = DBL_MAX;
    for (auto el : waypoint_candidates)
    {
      // ROS_INFO("closest_candidates : %d",el);
      double d = getPlaneDistance(getWaypointPosition(path,el), current_pose.position);
      if (d < distance_min)
      {
        waypoint_min = el;
        distance_min = d;
      }
    }
    return waypoint_min;
  }
  else
  {
    ROS_INFO("no candidate. search closest waypoint from all waypoints...");
    // if there is no candidate...
    int waypoint_min = -1;
    double distance_min = DBL_MAX;
    for (int i = 1; i < getSize(path); i++)
    {
      if (!isFront(path,i, current_pose))
        continue;

      // if (!wp.isValid(i, current_pose))
      //  continue;

      double d = getPlaneDistance(getWaypointPosition(path,i), current_pose.position);
      if (d < distance_min)
      {
        waypoint_min = i;
        distance_min = d;
      }
    }
    return waypoint_min;
  }
}

// let the linear equation be "ax + by + c = 0"
// if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
bool Utility::getLinearEquation(const geometry_msgs::Point &start,const geometry_msgs::Point &end, double &a, double &b, double &c)
{
  //(x1, y1) = (start.x, star.y), (x2, y2) = (end.x, end.y)
  double sub_x = fabs(start.x - end.x);
  double sub_y = fabs(start.y - end.y);
  double error = pow(10, -5); // 0.00001

  if (sub_x < error && sub_y < error)
  {
    ROS_INFO("two points are the same point!!");
    return false;
  }

  a = end.y - start.y;
  b = (-1) * (end.x - start.x);
  c = (-1) * (end.y - start.y) * start.x + (end.x - start.x) * start.y;

  return true;
}
double Utility::getDistanceBetweenLineAndPoint(const geometry_msgs::Point &point, double a, double b, double c)
{
  double d = fabs(a * point.x + b * point.y + c) / sqrt(pow(a, 2) + pow(b, 2));

  return d;
}

tf::Vector3 Utility::point2vector(geometry_msgs::Point point)
{
  tf::Vector3 vector(point.x, point.y, point.z);
  return vector;
}

geometry_msgs::Point Utility::vector2point(tf::Vector3 vector)
{
  geometry_msgs::Point point;
  point.x = vector.getX();
  point.y = vector.getY();
  point.z = vector.getZ();
  return point;
}

tf::Vector3 Utility::rotateUnitVector(tf::Vector3 unit_vector, double degree)
{
  tf::Vector3 w1(cos(deg2rad(degree)) * unit_vector.getX() - sin(deg2rad(degree)) * unit_vector.getY(),
                 sin(deg2rad(degree)) * unit_vector.getX() + cos(deg2rad(degree)) * unit_vector.getY(), 0);
  tf::Vector3 unit_w1 = w1.normalize();

  return unit_w1;
}

geometry_msgs::Point Utility::rotatePoint(geometry_msgs::Point point, double degree)
{
  geometry_msgs::Point rotate;
  rotate.x = cos(deg2rad(degree)) * point.x - sin(deg2rad(degree)) * point.y;
  rotate.y = sin(deg2rad(degree)) * point.x + cos(deg2rad(degree)) * point.y;

  return rotate;
}

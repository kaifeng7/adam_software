#ifndef __DDL_UTILS__
#define __DDL_UTILS__

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <vector>

#define PI 3.1415926
#define RAD2ANGLE(x) ((x)*57.295780)
#define ANGLE2RAD(x) ((x)*0.0174533)

namespace util
{

bool pubMarkerText(const ros::Publisher pub, const geometry_msgs::Pose pose, const ros::Time stamp, const std::string frame, const std::string text)
{
  visualization_msgs::Marker msg;
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.b = 0.0;
  color.r = 1.0;
  color.g = 1.0;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame;
  msg.ns = "~";
  msg.id = 0;
  msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  msg.action = visualization_msgs::Marker::ADD;
  msg.text = text;
  msg.pose = pose;
  msg.color = color; // yellow
  msg.scale.x = 0.;
  msg.scale.y = 5.;
  msg.scale.z = 1.;
  msg.frame_locked = true;
  pub.publish(msg);
  return true;
}

bool pubMarkerCylinder(const ros::Publisher pub, const geometry_msgs::Pose pose, const ros::Time stamp, const std::string frame, const geometry_msgs::Vector3 scale)
{
  visualization_msgs::Marker msg;
  std_msgs::ColorRGBA color;
  color.a = 0.4;
  color.b = 1.0;
  color.r = 0.0;
  color.g = 0.0;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame;
  msg.ns = "~";
  msg.id = 0;
  msg.type = visualization_msgs::Marker::SPHERE;
  msg.action = visualization_msgs::Marker::ADD;
  msg.pose = pose;
  msg.color = color;
  msg.scale = scale;
  msg.frame_locked = true;
  pub.publish(msg);
  return true;
}

bool pubMarkerCube(const ros::Publisher pub, const geometry_msgs::Pose pose, const ros::Time stamp, const std::string frame, const geometry_msgs::Vector3 scale)
{
  visualization_msgs::Marker msg;
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.b = 1.0;
  color.r = 1.0;
  color.g = 1.0;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame;
  msg.ns = "~";
  msg.id = 0;
  msg.type = visualization_msgs::Marker::CUBE;
  msg.action = visualization_msgs::Marker::ADD;
  msg.pose = pose;
  msg.color = color;
  msg.scale = scale;
  msg.frame_locked = true;
  pub.publish(msg);
}

bool pubMarkerLineList(const ros::Publisher pub, const geometry_msgs::Pose pose, const ros::Time stamp, const std::string frame, const std::vector<geometry_msgs::Point> points)
{
  visualization_msgs::Marker msg;
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.b = 0.0;
  color.r = 1.0;
  color.g = 0.0;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame;
  msg.ns = "~";
  msg.id = 0;
  msg.type = visualization_msgs::Marker::LINE_LIST;
  msg.action = visualization_msgs::Marker::ADD;
  msg.pose = pose;
  msg.color = color;
  msg.scale.x = 0.1;
  msg.scale.y = 0.1;
  msg.scale.z = 0.1;
  msg.frame_locked = true;
  msg.points.insert(msg.points.end(), points.begin(), points.end());
  pub.publish(msg);
}

bool equal(double a, double b, double tolerance)
{
  return std::fabs(a - b) < tolerance;
}

double cut(double x, double min, double max)
{
  assert(min <= max);
  if (x < min)
  {
    return min;
  }
  else if (x > max)
  {
    return max;
  }
  return x;
}

bool arrayParser(const std::string &s_in, std::vector<geometry_msgs::Point32> &v_out)
{
  using namespace std;
  v_out.clear();
  stringstream input_ss(s_in);
  int depth = 0;
  geometry_msgs::Point32 point;
  vector<vector<double> > vec;
  vector<double> current_vec;
  while (!!input_ss && !input_ss.eof())
  {
    switch (input_ss.peek())
    {
    case EOF:
      break;
    case '[':
      depth++;
      if (depth > 2)
      {
        v_out.clear();
        ROS_ERROR("array depth greater than 2");
        return false;
      }
      input_ss.get();
      current_vec.clear();
      break;
    case ']':
      depth--;
      if (depth < 0)
      {
        v_out.clear();
        ROS_ERROR("syntax error: more close ] than open [");
        return false;
      }
      input_ss.get();
      if (depth == 1)
      {
        vec.push_back(current_vec);
      }
      break;
    case ',':
    case ' ':
    case '\t':
    case '\n':
    case '\r':
      input_ss.get();
      break;
    default:
      if (depth != 2)
      {
        v_out.clear();
        ROS_ERROR("syntax error: num at depth other than 2. char was %c", char(input_ss.peek()));
        return false;
      }
      double value;
      input_ss >> value;
      if (!!input_ss)
      {
        current_vec.push_back(value);
      }
      break;
    }
  }

  if (depth != 0)
  {
    v_out.clear();
    ROS_ERROR("syntax error: unterminated vector string");
    return false;
  }

  for (int i = 0; i < vec.size(); i++)
  {
    if (vec[i].size() != 2)
    {
      v_out.clear();
      ROS_ERROR("points in the pwm2rpm/pwm2angle specification must be pairs of numbers, found a point with %d numbers!", vec[i].size());
      return false;
    }
    else
    {
      point.x = vec[i][0];
      point.y = vec[i][1];
      v_out.push_back(point);
    }
  }
  return true;
}

  double calcDiffForRadian(const double &l, const double &r){
    double diff = l-r;
    if (diff >= M_PI){
      diff -= 2*M_PI;
    } else if (diff < -M_PI){
      diff += 2*M_PI;
    }
    return diff;
  }
} // namespace util

#endif
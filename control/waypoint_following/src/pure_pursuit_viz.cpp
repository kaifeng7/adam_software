/*
 * @Author: fengk 
 * @Date: 2019-04-15 22:21:42 
 * @Last Modified by:   fengk 
 * @Last Modified time: 2019-04-15 22:21:42 
 */


#include "pure_pursuit.h"

// display the next waypoint by markers.
visualization_msgs::Marker PurePursuit::displayNextWaypoint(geometry_msgs::Point position)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "next_waypoint_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = position;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.frame_locked = true;
  return marker;
}

// display the next target by markers.
visualization_msgs::Marker PurePursuit::displayNextTarget(geometry_msgs::Point target)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "next_target_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = target;
  std_msgs::ColorRGBA green;
  green.a = 1.0;
  green.b = 0.0;
  green.r = 0.0;
  green.g = 1.0;
  marker.color = green;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.frame_locked = true;
  return marker;
}

double PurePursuit::calcRadius(geometry_msgs::Point target, geometry_msgs::Pose current_pose)
{
  double radius;
  double denominator = 2 * utility.calcRelativeCoordinate(target, current_pose).y;
  double numerator = pow(utility.getPlaneDistance(target, current_pose.position), 2);

  if (denominator != 0)
    radius = numerator / denominator;
  else
    radius = 0;

  // ROS_INFO("radius : %lf", radius);
  return radius;
}

// generate the locus of pure pursuit
std::vector<geometry_msgs::Point> PurePursuit::generateTrajectoryCircle(geometry_msgs::Point target,
                                                           geometry_msgs::Pose current_pose)
{
  std::vector<geometry_msgs::Point> traj_circle_array;
  double radius = calcRadius(target, current_pose);
  double range = M_PI / 8;
  double increment = 0.01;

  for (double i = 0; i < range; i += increment)
  {
    // calc a point of circumference
    geometry_msgs::Point p;
    p.x = radius * cos(i);
    p.y = radius * sin(i);

    // transform to (radius,0)
    geometry_msgs::Point relative_p;
    relative_p.x = p.x - radius;
    relative_p.y = p.y;

    // rotate -90°
    geometry_msgs::Point rotate_p = utility.rotatePoint(relative_p, -90);

    // transform to vehicle plane
    geometry_msgs::Point tf_p = utility.calcAbsoluteCoordinate(rotate_p, current_pose);

    traj_circle_array.push_back(tf_p);
  }

  // reverse vector
  std::reverse(traj_circle_array.begin(), traj_circle_array.end());

  for (double i = 0; i > (-1) * range; i -= increment)
  {
    // calc a point of circumference
    geometry_msgs::Point p;
    p.x = radius * cos(i);
    p.y = radius * sin(i);

    // transform to (radius,0)
    geometry_msgs::Point relative_p;
    relative_p.x = p.x - radius;
    relative_p.y = p.y;

    // rotate -90°
    geometry_msgs::Point rotate_p = utility.rotatePoint(relative_p, -90);

    // transform to vehicle plane
    geometry_msgs::Point tf_p = utility.calcAbsoluteCoordinate(rotate_p, current_pose);

    traj_circle_array.push_back(tf_p);
  }

  return traj_circle_array;
}
// display the locus of pure pursuit by markers.
visualization_msgs::Marker PurePursuit::displayTrajectoryCircle(std::vector<geometry_msgs::Point> traj_circle_array)
{
  visualization_msgs::Marker traj_circle;
  traj_circle.header.frame_id = "map";
  traj_circle.header.stamp = ros::Time();
  traj_circle.ns = "trajectory_circle_marker";
  traj_circle.id = 0;
  traj_circle.type = visualization_msgs::Marker::LINE_STRIP;
  traj_circle.action = visualization_msgs::Marker::ADD;

  std_msgs::ColorRGBA white;
  white.a = 1.0;
  white.b = 1.0;
  white.r = 1.0;
  white.g = 1.0;
  //
  for (auto el : traj_circle_array)
    for (std::vector<geometry_msgs::Point>::iterator it = traj_circle_array.begin(); it != traj_circle_array.end();
         it++)
    {
      // traj_circle.points.push_back(*it);
      traj_circle.points.push_back(el);
      traj_circle.colors.push_back(white);
    }

  traj_circle.scale.x = 0.1;
  traj_circle.color.a = 0.3;
  traj_circle.color.r = 1.0;
  traj_circle.color.g = 0.0;
  traj_circle.color.b = 0.0;
  traj_circle.frame_locked = true;
  return traj_circle;
}

// display the search radius by markers.
visualization_msgs::Marker PurePursuit::displaySearchRadius(geometry_msgs::Point current_pose, double search_radius)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "search_radius_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = current_pose;
  marker.scale.x = search_radius * 2;
  marker.scale.y = search_radius * 2;
  marker.scale.z = 1.0;
  marker.color.a = 0.1;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.frame_locked = true;
  return marker;
}

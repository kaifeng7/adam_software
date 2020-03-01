/*
 * @Author: fengk 
 * @Date: 2019-04-12 14:53:33 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-16 10:27:20
 */


#ifndef PURE_PURSUIT_VIZ_H
#define PURE_PURSUIT_VIZ_H

// ROS includes
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <visualization_msgs/Marker.h>

// C++ includes
#include <memory>

// User defined includes

// display the next waypoint by markers.
visualization_msgs::Marker displayNextWaypoint(geometry_msgs::Point position);
// display the next target by markers.
visualization_msgs::Marker displayNextTarget(geometry_msgs::Point target);

double calcRadius(geometry_msgs::Point target, geometry_msgs::Pose current_pose);

// generate the locus of pure pursuit
std::vector<geometry_msgs::Point> generateTrajectoryCircle(geometry_msgs::Point target,
                                                           geometry_msgs::Pose current_pose);
// display the locus of pure pursuit by markers.
visualization_msgs::Marker displayTrajectoryCircle(std::vector<geometry_msgs::Point> traj_circle_array);

// display the search radius by markers.
visualization_msgs::Marker displaySearchRadius(geometry_msgs::Point current_pose, double search_radius);

#endif  // PURE_PURSUIT_VIZ_H

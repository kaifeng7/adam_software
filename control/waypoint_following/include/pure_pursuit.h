
/*
 * @Author: fengk 
 * @Date: 2019-04-12 14:54:53 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-17 17:51:55
 */

#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

// ROS includes

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>

#include <nav_msgs/Path.h>
#include <adam_msgs/VehicleCmd.h>
#include <adam_msgs/CarBasicInfo.h>
#include <adam_msgs/CarLimitInfo.h>


// User defined includes
#include "utility.h"
#include "pure_pursuit_viz.h"

class PurePursuit
{
public:
  PurePursuit();
  ~PurePursuit();

  void MainLoop();
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  // constant
  int LOOP_RATE_; // processing frequency

  const double RADIUS_MAX;
  const double KAPPA_MIN;

  // variables
  bool is_linear_interpolation;
  bool bPath, bCurrentPose;
  bool bCarBasicInfo,bCarLimitInfo;
  bool bCmdVel;

  double lookahead_distance_ratio_;
  double minimum_lookahead_distance_; // the next waypoint must be outside of this threshold.

  double m_LookaheadDistance;
  double m_CurrentVelocity;
  double m_CommandVelocity;
  adam_msgs::CarBasicInfo m_CarBasicInfo;
  adam_msgs::CarLimitInfo m_CarLimitInfo;

  geometry_msgs::Pose m_CurrentPose;
  nav_msgs::Path m_Path;
  geometry_msgs::Point m_NextTargetPosition;

  Utility utility;

  // callbacks
  void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg);
  void callbackFromPath(const nav_msgs::PathConstPtr &msg);
  void callbackFromVelocity(const geometry_msgs::TwistConstPtr &msg);
  void callbackGetCarBasicInfo(const adam_msgs::CarBasicInfoConstPtr &msg);
  void callbackGetCarLimitInfo(const adam_msgs::CarLimitInfoConstPtr &msg);




  // publisher
  ros::Publisher pub_VehicleCmd;
  ros::Publisher pub_NextWaypoint;
  ros::Publisher pub_NextTargetPosition;
  ros::Publisher pub_SearchRadius;
  ros::Publisher pub_TrajectoryCircle;
  ros::Publisher pub_AngularGravity;
  ros::Publisher pub_DeviationCurrentPosition;

  // subscriber
  ros::Subscriber sub_Path;
  ros::Subscriber sub_CurrentPose;
  ros::Subscriber sub_CommandVelocity;
  ros::Subscriber sub_CarBasicInfo;
  ros::Subscriber sub_CarLimitInfo;

  // initializer
  void initForROS();

  // functions
  void publishTwist(const bool &can_get_curvature, const double &kappa);
  void publishDeviationCurrentPosition(const geometry_msgs::Point &point, const nav_msgs::Path &path);

  double computeAngularGravity(double velocity, double kappa);
  double calcCurvature(const geometry_msgs::Point &target);
  bool interpolateNextTarget(int next_waypoint, geometry_msgs::Point *next_target);
  int getNextWaypoint();
  bool IsGetCurvature(int &i_next_waypoint, double &output_kappa);
  double LookaheadDistance();

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
};

#endif // PURE_PURSUIT_H

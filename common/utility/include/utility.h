/*
 * @Author: fengk 
 * @Date: 2019-04-09 23:02:14 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-16 22:20:34
 */

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <adam_msgs/WayPointStamped.h>
#include <nav_msgs/Path.h>

class Utility
{
  public:
	double FixNegativeAngle(const double &a);
	double AngleBetweenTwoAnglesPositive(const double &a1, const double &a2);

	double PointLength(geometry_msgs::Point p);
	double AngleBetweenTwoPoints(geometry_msgs::Point from, geometry_msgs::Point to);

	int getSize(const nav_msgs::Path &path);
	bool isEmpty(const nav_msgs::Path &path);

	

	
	double getInterval(const nav_msgs::Path &path);
	geometry_msgs::Point getWaypointPosition(const nav_msgs::Path &path,const int &i_waypoint);
	geometry_msgs::Quaternion getWaypointOrientation(const nav_msgs::Path &path,const int &i_waypoint);
	geometry_msgs::Pose getWaypointPose(const nav_msgs::Path &path,const int &i_waypoint);

	bool isFront(const nav_msgs::Path &path,const int &i_waypoint, const geometry_msgs::Pose &current_pose);

	tf::Vector3 point2vector(geometry_msgs::Point point);						 // convert point to vector
	geometry_msgs::Point vector2point(tf::Vector3 vector);						 // convert vector to point
	tf::Vector3 rotateUnitVector(tf::Vector3 unit_vector, double degree);		 // rotate unit vector by degree
	geometry_msgs::Point rotatePoint(geometry_msgs::Point point, double degree); // rotate point vector by degree

	double DecelerateVelocity(double distance, double prev_velocity); 
	double Deceleration(double distance,double prev_velocity);

	geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Point point,
												geometry_msgs::Pose current_pose); // transform point into the coordinate
																				   // of current_pose
	geometry_msgs::Point calcAbsoluteCoordinate(geometry_msgs::Point point,
												geometry_msgs::Pose current_pose);					   // transform point into the global
																									   // coordinate
	double getPlaneDistance(const geometry_msgs::Point &target1, const geometry_msgs::Point &target2); // get 2 dimentional distance between target 1 and target 2
	double getPlaneDistance2(const geometry_msgs::Point &target1, const geometry_msgs::Point &target2); // get 2 dimentional distance between target 1 and target 2

	int getClosestWaypoint(const nav_msgs::Path &current_path, const geometry_msgs::Pose &current_pose);
	bool getLinearEquation(const geometry_msgs::Point &start, const geometry_msgs::Point &end, double &a, double &b, double &c);
	double getDistanceBetweenLineAndPoint(const geometry_msgs::Point &point, double sa, double b, double c);
	double getRelativeAngle(const geometry_msgs::Pose &waypoint_pose,const geometry_msgs::Pose &vehicle_pose);

	inline double deg2rad(double deg)
	{
		return deg * M_PI / 180.0;
	} // convert degree to radian

	inline double rad2deg(double rad)
	{
		return rad * 180.0 / M_PI;
	}
	inline double kmph2mps(double velocity_kmph)
	{
		return (velocity_kmph * 1000) / (60 * 60);
	}
	inline double mps2kmph(double velocity_mps)
	{
		return (velocity_mps * 60 * 60) / 1000;
	}
};

class Mat3
{
	double m[3][3];

  public:
	Mat3()
	{
		//initialize Identity by default
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				m[i][j] = 0;

		m[0][0] = m[1][1] = m[2][2] = 1;
	}

	Mat3(double transX, double transY, bool mirrorX, bool mirrorY)
	{
		m[0][0] = (mirrorX == true) ? -1 : 1;
		m[0][1] = 0;
		m[0][2] = transX;
		m[1][0] = 0;
		m[1][1] = (mirrorY == true) ? -1 : 1;
		m[1][2] = transY;
		m[2][0] = 0;
		m[2][1] = 0;
		m[2][2] = 1;
	}

	Mat3(double transX, double transY)
	{
		m[0][0] = 1;
		m[0][1] = 0;
		m[0][2] = transX;
		m[1][0] = 0;
		m[1][1] = 1;
		m[1][2] = transY;
		m[2][0] = 0;
		m[2][1] = 0;
		m[2][2] = 1;
	}

	Mat3(double rotation_angle)
	{
		double c = cos(rotation_angle);
		double s = sin(rotation_angle);
		m[0][0] = c;
		m[0][1] = -s;
		m[0][2] = 0;
		m[1][0] = s;
		m[1][1] = c;
		m[1][2] = 0;
		m[2][0] = 0;
		m[2][1] = 0;
		m[2][2] = 1;
	}

	Mat3(adam_msgs::WayPoint rotationCenter)
	{
		double c = cos(tf::getYaw(rotationCenter.ps.pose.orientation));
		double s = sin(tf::getYaw(rotationCenter.ps.pose.orientation));
		double u = rotationCenter.ps.pose.position.x;
		double v = rotationCenter.ps.pose.position.y;
		m[0][0] = c;
		m[0][1] = -s;
		m[0][2] = -u * c + v * s + u;
		m[1][0] = s;
		m[1][1] = c;
		m[1][2] = -u * s - v * c + v;
		m[2][0] = 0;
		m[2][1] = 0;
		m[2][2] = 1;
	}

	adam_msgs::WayPoint operator*(adam_msgs::WayPoint v)
	{
		adam_msgs::WayPoint _v = v;
		v.ps.pose.position.x = m[0][0] * _v.ps.pose.position.x + m[0][1] * _v.ps.pose.position.y + m[0][2] * 1;
		v.ps.pose.position.y = m[1][0] * _v.ps.pose.position.x + m[1][1] * _v.ps.pose.position.y + m[1][2] * 1;
		return v;
	}
};
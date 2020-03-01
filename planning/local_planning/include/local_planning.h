/*
 * @Author: fengk 
 * @Date: 2019-04-02 21:37:21 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-16 21:33:17
 */

#ifndef LOCAL_PLANNING_H
#define LOCAL_PLANNING_H

#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <iomanip>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <adam_msgs/WayPoint.h>
#include <adam_msgs/CarLimitInfo.h>
#include <adam_msgs/CarBasicInfo.h>

#include "utility.h"


class PlanningParams
{
  public:
	double maxSpeed;
	double moreSpeed;
	double lessSpeed;
	double minSpeed;

	double maxAngle;
	double midAngle;
	double minAngle;
	
	double carTipMargin;
	double rollInMargin;
	double rollInSpeedFactor;
	double pathDensity;
	double lastInterpolationNumber;

	double rollOutDensity;
	int rollOutNumber;

	double planDistance;
	double rollOutDistance;

	double laneChangeSpeedFactor;

	double smoothingDataWeight;
	double smoothingSmoothWeight;
	double smoothingToleranceError;

	int obstacleThreshold;

	double weightPriority;//权重
	double weightTransition;
	double weightObj;

};


class RelativeInfo
{
  public:
	double perp_distance;
	double to_front_distance; //negative
	double from_back_distance;
	int iFront;
	int iBack;
	int iGlobalPath;
	adam_msgs::WayPoint perp_point;
	double angle_diff; // degrees
	bool bBefore;
	bool bAfter;
	double after_angle;

	RelativeInfo()
	{
		after_angle = 0;
		bBefore = false;
		bAfter = false;
		perp_distance = 0;
		to_front_distance = 0;
		from_back_distance = 0;
		iFront = 0;
		iBack = 0;
		iGlobalPath = 0;
		angle_diff = 0;
	}
};



class TrajectoryCost
{
  public:
	int index;
	int relative_index;
	double closest_obj_velocity;
	double distance_from_center;
	double priority_cost;	//0 to 1
	double transition_cost;  // 0 to 1
	double closest_obj_cost; // 0 to 1
	bool obj;
	double cost;
	double closest_obj_distance;

	bool bBlocked;

	TrajectoryCost()
	{
		index = -1;
		relative_index = -100;
		closest_obj_velocity = 0;
		priority_cost = 0;
		transition_cost = 0;
		closest_obj_cost = 0;
		distance_from_center = 0;
		cost = 0;
		closest_obj_distance = -1;

		bBlocked = false;
	}
};

class TrajectoryGen
{
  public:
	PlanningParams m_PlanningParams;
	adam_msgs::CarBasicInfo m_CarBasicInfo;
	adam_msgs::CarLimitInfo m_CarLimitInfo;

	geometry_msgs::Pose m_InitPos;
	bool bInitPos;
	bool bCurrentPos;
	bool bVehicleStatus;
	bool bCostMap;	
	bool bGlobalPath;
	
	bool bLocalPath;

	bool bCarBasicInfo;
	bool bCarLimitInfo;

	adam_msgs::WayPoint m_CurrentPos;
	

	double m_Speed;
	double m_Steer;



	std::vector<adam_msgs::WayPoint> m_temp_path;
	std::vector<adam_msgs::WayPoint> m_GlobalPath;
	std::vector<adam_msgs::WayPoint> m_GlobalPathSection;

	std::vector<std::vector<adam_msgs::WayPoint> > m_RollOuts;


	nav_msgs::MapMetaData map_info_;//地图
	std::vector<std::vector<bool> > is_Obstacle_nodes;

	int m_PrevCostIndex;
	std::vector<TrajectoryCost> m_TrajectoryCosts;

	Utility utility;

	ros::NodeHandle nh_,pnh_;

	//define publishers
	ros::Publisher pub_LocalTrajectories;
	ros::Publisher pub_LocalTrajectoriesSpeed;
	ros::Publisher pub_LocalTrajectoriesRviz;
	ros::Publisher pub_LocalTrajectoriesSpeedRviz;
	ros::Publisher pub_LocalTrajectoriesCurrentSpeedRviz;

	ros::Publisher pub_CollisionPointsRviz;
	ros::Publisher pub_LocalWeightedTrajectoriesRviz;
	ros::Publisher pub_LocalWeightedTrajectories;
	ros::Publisher pub_TrajectoryCost;
	ros::Publisher pub_CarModelRviz;
	
	ros::Publisher pub_smoothpath;

	// define subscribers.
	ros::Subscriber sub_initialpose;
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_current_velocity;
	//ros::Subscriber sub_robot_odom;
	ros::Subscriber sub_GlobalPlannerPaths;
	ros::Subscriber sub_ObstacleMap;

	ros::Subscriber sub_CarLimitInfo;
	ros::Subscriber sub_CarBasicInfo;

	// Callback function for subscriber.
	void callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &input);
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg);
	//void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr &msg);
	void callbackGetGlobalPlannerPath(const nav_msgs::PathConstPtr &msg);
	void callbackGetObstaclesMap(const nav_msgs::OccupancyGridConstPtr &msg);
	void callbackGetCarBasicInfo(const adam_msgs::CarBasicInfoConstPtr &msg);
	void callbackGetCarLimitInfo(const adam_msgs::CarLimitInfoConstPtr &msg);


	//Helper Functions
	double CalcAngleAndCost(std::vector<adam_msgs::WayPoint> &path);
	int ExtractPartFromPointToDistanceDirectionFast(const std::vector<adam_msgs::WayPoint> &originalPath, 
													 const adam_msgs::WayPoint &pos, 
													 std::vector<adam_msgs::WayPoint> &extractedPath);
	int GetClosestNextPointIndexDirectionFast(const std::vector<adam_msgs::WayPoint> &trajectory, 
											  const adam_msgs::WayPoint &p, 
											  const int &prevIndex);
	bool CompareTrajectories(const std::vector<adam_msgs::WayPoint> &path1, const std::vector<adam_msgs::WayPoint> &path2);
	void FixPathDensity(std::vector<adam_msgs::WayPoint> &path, const double &distanceDensity);
	void GenerateRunoffTrajectory(const std::vector<adam_msgs::WayPoint> &referencePath, 
								  const adam_msgs::WayPoint &carPos,
								  const double &speed, 
								  std::vector<std::vector<adam_msgs::WayPoint> > &rollOutPath);
	void CalculateRollInTrajectories(const adam_msgs::WayPoint &carPos, 
									 const double &speed, 
									 const std::vector<adam_msgs::WayPoint> &originalCenter, 
									 int &start_index,
									 int &end_index, 
									 std::vector<double> &end_laterals, 
									 std::vector<std::vector<adam_msgs::WayPoint> > &rollInPaths);

	void SmoothPath(std::vector<adam_msgs::WayPoint> &path, 
					double weight_data, 
					double weight_smooth, 
					double tolerance);

	bool GetRelativeInfo(const std::vector<adam_msgs::WayPoint> &trajectory, 
						 const adam_msgs::WayPoint &p, 
						 RelativeInfo &info, 
						 const int &prevIndex);

	void TrajectoriesToMarkers(const std::vector<std::vector<adam_msgs::WayPoint> > &paths, 
							   visualization_msgs::MarkerArray &markerArray,
							   const TrajectoryCost &tc);
	void TrajectoriesSpeedToMarkers(const std::vector<std::vector<adam_msgs::WayPoint> > &paths, 
							   visualization_msgs::MarkerArray &markerArray,
							   const TrajectoryCost &tc);
	void CurrentSpeedToMarkers(const double &vel, visualization_msgs::Marker &marker);
	
	void CarModelToMarker(visualization_msgs::Marker &marker);


	std::vector<TrajectoryCost> CalculatePriorityCosts(const std::vector<std::vector<adam_msgs::WayPoint> > &laneRollOuts);
	void NormalizeCosts(std::vector<TrajectoryCost> &trajectoryCosts);
	void CalculateTransitionCosts(std::vector<TrajectoryCost> &trajectoryCosts, const int &currTrajectoryIndex);
	void poseToIndex(const geometry_msgs::Pose &pose, int *index_x, int *index_y);
	bool isOutOfRange(const int index_x, const int index_y);
	bool detectCollision(const adam_msgs::WayPoint &wp);


	void CalculateObstacleCosts(std::vector<TrajectoryCost> &trajectoryCosts, 
								const std::vector<std::vector<adam_msgs::WayPoint> > &rollOuts,
								const adam_msgs::WayPoint &m_CurrentPos);

	TrajectoryCost DoOneStep(const std::vector<std::vector<adam_msgs::WayPoint> > &rollOuts, 
							 const std::vector<adam_msgs::WayPoint> &totalPaths,
							 const adam_msgs::WayPoint &currState,
							 const int &currTrajectoryIndex, 
							 const double &m_Steer);

	TrajectoryGen();
	~TrajectoryGen();
	void initROS();
	void MainLoop();
};



#endif

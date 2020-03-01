/*
 * @Author: fengk 
 * @Date: 2019-04-09 22:52:35 
 * @Last Modified by:   fengk 
 * @Last Modified time: 2019-04-09 22:52:35 
 */
#include "local_planning.h"

void TrajectoryGen::TrajectoriesToMarkers(const std::vector<std::vector<adam_msgs::WayPoint>> &paths, 
										  visualization_msgs::MarkerArray &markerArray, 
										  const TrajectoryCost &tc)
{
	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	lane_waypoint_marker.ns = "global_lane_array_marker";
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;

	lane_waypoint_marker.frame_locked = false;


	int count = 0;

	for (unsigned int i = 0; i < paths.size(); i++)
	{
		lane_waypoint_marker.points.clear();
		lane_waypoint_marker.id = count;

		for (unsigned int j = 0; j < paths.at(i).size(); j++)
		{
			geometry_msgs::Point point;

			point.x = paths.at(i).at(j).ps.pose.position.x;
			point.y = paths.at(i).at(j).ps.pose.position.y;
			point.z = paths.at(i).at(j).ps.pose.position.z;

			lane_waypoint_marker.points.push_back(point);
	
		}

		lane_waypoint_marker.scale.x = 0.1;
		lane_waypoint_marker.scale.y = 0.1;
		lane_waypoint_marker.color.a = 0.9;
		if (m_TrajectoryCosts.at(i).obj == true)
		{
			lane_waypoint_marker.color.r = 1.0; //red;
			lane_waypoint_marker.color.g = 0.0;
			lane_waypoint_marker.color.b = 0.0;
			lane_waypoint_marker.color.a = 0.5;
		}
		else if (i == tc.index)
		{
			lane_waypoint_marker.color.r = 0.0;
			lane_waypoint_marker.color.g = 1.0;
			lane_waypoint_marker.color.b = 0.0;
		}
		else
		{
			lane_waypoint_marker.color.r = 0.0;
			lane_waypoint_marker.color.g = 0.0;
			lane_waypoint_marker.color.b = 1 - m_TrajectoryCosts.at(i).cost * 5;
		}

		markerArray.markers.push_back(lane_waypoint_marker);
		count++;
	}
}

void TrajectoryGen::TrajectoriesSpeedToMarkers(const std::vector<std::vector<adam_msgs::WayPoint>> &paths, 
										  visualization_msgs::MarkerArray &markerArray, 
										  const TrajectoryCost &tc)
{

	visualization_msgs::Marker lane_speed_marker;
	lane_speed_marker.header.stamp = ros::Time();
	lane_speed_marker.header.frame_id = "map";
	lane_speed_marker.ns = "global_speed_array_marker";
	lane_speed_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	lane_speed_marker.action = visualization_msgs::Marker::ADD;
	lane_speed_marker.color.a = 0.9; // yellow
	lane_speed_marker.color.r = 1.0;
	lane_speed_marker.color.g = 1.0;
	lane_speed_marker.color.b = 0.0;
	lane_speed_marker.scale.x = 0.0;
	lane_speed_marker.scale.y = 0.0;
	lane_speed_marker.scale.z = 0.2;
	lane_speed_marker.frame_locked = false;
	int count = 0;

	for (unsigned int i = 0; i < paths.size(); i++)
	{

		for (unsigned int j = 0; j < paths.at(i).size(); j++)
		{
			lane_speed_marker.pose = paths.at(i).at(j).ps.pose;
			lane_speed_marker.id = count;
			char str[256];
			sprintf(str, "%.2lf", paths.at(i).at(j).vel_x);
			std::string result = str;
			lane_speed_marker.text = result;
			markerArray.markers.push_back(lane_speed_marker);
			count++;
		}
	}
}

void TrajectoryGen::CurrentSpeedToMarkers(const double &vel, visualization_msgs::Marker &marker)
{

	visualization_msgs::Marker current_speed_marker;
	current_speed_marker.header.stamp = ros::Time();
	current_speed_marker.header.frame_id = "map";
	current_speed_marker.ns = "current_speed_marker";
	current_speed_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	current_speed_marker.action = visualization_msgs::Marker::ADD;
	current_speed_marker.color.a = 0.9; // yellow
	current_speed_marker.color.r = 0.0;
	current_speed_marker.color.g = 1.0;
	current_speed_marker.color.b = 1.0;
	current_speed_marker.scale.x = 0.0;
	current_speed_marker.scale.y = 0.0;
	current_speed_marker.scale.z = 2.0;
	current_speed_marker.frame_locked = false;
	current_speed_marker.pose.position.x = 0.0;
	current_speed_marker.pose.position.y = -20.0;
	current_speed_marker.id = 0;

	char str[256];
	sprintf(str, "Speed:%.2lf", vel);
	std::string result = str;
	current_speed_marker.text = result;
	marker = current_speed_marker;
}

void TrajectoryGen::CarModelToMarker(visualization_msgs::Marker &marker)
{
	visualization_msgs::Marker car_model_marker;
	car_model_marker.header.stamp = ros::Time();
	car_model_marker.header.frame_id = "map";
	car_model_marker.ns = "car_model_marker";
	car_model_marker.type = visualization_msgs::Marker::CUBE;
	car_model_marker.action = visualization_msgs::Marker::ADD;
	car_model_marker.color.a = 0.9;
	car_model_marker.color.g = 0;
	car_model_marker.color.b = 0;
	car_model_marker.color.r = 0;
	car_model_marker.scale.x = m_CarBasicInfo.length;
	car_model_marker.scale.y = m_CarBasicInfo.width;
	car_model_marker.scale.z = 1.5;
	car_model_marker.frame_locked = false;
	car_model_marker.pose = m_CurrentPos.ps.pose;
	car_model_marker.id = 0;
	marker = car_model_marker;	

}

/*
 * @Author: fengk 
 * @Date: 2019-04-03 17:32:26 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-03 18:52:52
 */

#include "load_vectormap.h"


visualization_msgs::MarkerArray LoadMap::StartToMarkers(std::vector<std::vector<adam_msgs::WayPoint> > all_paths)
{
  visualization_msgs::MarkerArray all_start_markers;

  visualization_msgs::Marker start_waypoint_marker;
  start_waypoint_marker.header.frame_id = "map";
  start_waypoint_marker.header.stamp = ros::Time();
  start_waypoint_marker.ns = "all_start_array_marker";
  start_waypoint_marker.type = visualization_msgs::Marker::SPHERE;
  start_waypoint_marker.action = visualization_msgs::Marker::ADD;
  start_waypoint_marker.color.a = 0.9;

  start_waypoint_marker.scale.x = 1.0;
  start_waypoint_marker.scale.y = 1.0;
  start_waypoint_marker.scale.z = 1.0;
  start_waypoint_marker.frame_locked = false;

  int count = 0;

  for (unsigned int i = 0; i < all_paths.size(); i++)
  {
    start_waypoint_marker.points.clear();
    start_waypoint_marker.id = count;
    start_waypoint_marker.color.r = 0.0;
    start_waypoint_marker.color.g = 0.0;
    start_waypoint_marker.color.b = 1.0;

    geometry_msgs::Pose p;

    p.position.x = all_paths.at(i).at(0).ps.pose.position.x;
    p.position.y = all_paths.at(i).at(0).ps.pose.position.y;
    p.position.z = all_paths.at(i).at(0).ps.pose.position.z;

    start_waypoint_marker.pose = p;

    all_start_markers.markers.push_back(start_waypoint_marker);
    count++;

  }
  return all_start_markers;
}

visualization_msgs::MarkerArray LoadMap::TrajToMarkers(adam_msgs::Road all_paths)
{
  visualization_msgs::MarkerArray all_traj_markers;

  visualization_msgs::Marker traj_waypoint_marker;
  traj_waypoint_marker.header.frame_id = "map";
  traj_waypoint_marker.header.stamp = ros::Time();
  traj_waypoint_marker.ns = "all_traj_array_marker";
  traj_waypoint_marker.type = visualization_msgs::Marker::SPHERE;
  traj_waypoint_marker.action = visualization_msgs::Marker::ADD;
  traj_waypoint_marker.color.a = 0.9;

  traj_waypoint_marker.scale.x = 0.1;
  traj_waypoint_marker.scale.y = 0.1;
  traj_waypoint_marker.scale.z = 0.1;
  traj_waypoint_marker.frame_locked = false;

  int count = 0;

  for (unsigned int i = 0; i < all_paths.lanes.size(); i++)
  {
    for (unsigned int j = 0; j < all_paths.lanes.at(i).wps.size(); j++)
    {
      traj_waypoint_marker.points.clear();
      traj_waypoint_marker.id = count;
      traj_waypoint_marker.color.r = 0.0;
      traj_waypoint_marker.color.g = 0.0;
      traj_waypoint_marker.color.b = 1.0;

      geometry_msgs::Pose p;

      p.position.x = all_paths.lanes.at(i).wps.at(j).ps.pose.position.x;
      p.position.y = all_paths.lanes.at(i).wps.at(j).ps.pose.position.y;
      p.position.z = all_paths.lanes.at(i).wps.at(j).ps.pose.position.z;

      traj_waypoint_marker.pose = p;

      all_traj_markers.markers.push_back(traj_waypoint_marker);
      count++;
    }

  }
  return all_traj_markers;
}

visualization_msgs::MarkerArray LoadMap::PathToMarkers(std::vector<std::vector<adam_msgs::WayPoint> > all_paths)
{
  visualization_msgs::MarkerArray all_paths_markers;

  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time();
  lane_waypoint_marker.ns = "all_paths_array_marker";
  lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.color.a = 0.9; // yellow

  lane_waypoint_marker.scale.x = 0.1;
  lane_waypoint_marker.scale.y = 0.1;
  lane_waypoint_marker.scale.z = 0.0;
  lane_waypoint_marker.frame_locked = false;

  int count = 0;

  for (unsigned int i = 0; i < all_paths.size(); i++)
  {
    lane_waypoint_marker.points.clear();
    lane_waypoint_marker.id = count;
    lane_waypoint_marker.color.r = 0.0;
    lane_waypoint_marker.color.g = 1.0;
    lane_waypoint_marker.color.b = 0.0;
    for (unsigned int j = 0; j < all_paths.at(i).size(); j++)
    {
      geometry_msgs::Point point;

      point.x = all_paths.at(i).at(j).ps.pose.position.x;
      point.y = all_paths.at(i).at(j).ps.pose.position.y;
      point.z = all_paths.at(i).at(j).ps.pose.position.z;

      lane_waypoint_marker.points.push_back(point);
    }

    all_paths_markers.markers.push_back(lane_waypoint_marker);
    count++;
  }
  return all_paths_markers;
}

visualization_msgs::MarkerArray LoadMap::LaneToMarkers(std::vector<std::vector<std::vector<std::vector<adam_msgs::WayPoint> > > > all_paths)
{
  visualization_msgs::MarkerArray all_lanes_markers;

  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time();
  lane_waypoint_marker.ns = "all_lanes_array_marker";
  lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.color.a = 0.9;

  lane_waypoint_marker.scale.x = 0.1;
  lane_waypoint_marker.scale.y = 0.1;
  lane_waypoint_marker.scale.z = 0.1;
  lane_waypoint_marker.frame_locked = false;

  int count = 0;

  for (unsigned int i = 0; i < all_paths.size(); i++)
  {
    for (unsigned int j = 0; j < all_paths.at(i).size(); j++)
    {
      for (unsigned int k = 0; k < all_paths.at(i).at(j).size(); k++)
      {
        lane_waypoint_marker.points.clear();
        lane_waypoint_marker.color.r = 1.0;
        lane_waypoint_marker.color.g = 0.0;
        lane_waypoint_marker.color.b = 0.0;          
        lane_waypoint_marker.id = count;

        for (unsigned int l = 0; l < all_paths.at(i).at(j).at(k).size(); l++)
        {
          geometry_msgs::Point p;

          p.x = all_paths.at(i).at(j).at(k).at(l).ps.pose.position.x;
          p.y = all_paths.at(i).at(j).at(k).at(l).ps.pose.position.y;
          p.z = all_paths.at(i).at(j).at(k).at(l).ps.pose.position.z;

          lane_waypoint_marker.points.push_back(p); 
          
        }
        count++;
        all_lanes_markers.markers.push_back(lane_waypoint_marker);

      }
    }
  }
  return all_lanes_markers;
}

visualization_msgs::MarkerArray LoadMap::NumberToMarkers(adam_msgs::Road trajs)
{
  visualization_msgs::MarkerArray all_number_markers;

  visualization_msgs::Marker number_marker;
  number_marker.header.frame_id = "map";
  number_marker.header.stamp = ros::Time();
  number_marker.ns = "all_number_array_marker";
  number_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  number_marker.action = visualization_msgs::Marker::ADD;
  number_marker.color.a = 0.9;

	number_marker.color.r = 1.0;
	number_marker.color.g = 1.0;
	number_marker.color.b = 0.0;
	number_marker.scale.x = 0.0;
	number_marker.scale.y = 0.0;
	number_marker.scale.z = 5;
  number_marker.frame_locked = false;

  int count = 0;

  for (unsigned int i = 0; i < trajs.lanes.size(); i++)
  {
    number_marker.pose = trajs.lanes.at(i).wps.at(trajs.lanes.at(i).wps.size()/2).ps.pose;
    number_marker.id = count;

    
    count++;
    char str[256];
		sprintf(str, "%d", i);
    std::string result = str;
    number_marker.text = result;

    all_number_markers.markers.push_back(number_marker);
  }
  return all_number_markers;
}
/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>
#include <obj_map/Points2CostmapConfig.h>
#include <utility>
#include <tf/transform_datatypes.h>

namespace
{
double HEIGHT_LIMIT = 20; // from sensor
double CAR_LENGTH = 0.5;
double CAR_WIDTH = 0.5;

ros::Publisher g_costmap_pub;
double g_resolution;
int g_cell_width;
int g_cell_height;
double g_map_width;
double g_map_height;
double g_offset_x;
double g_offset_y;
double g_offset_z;
double g_roll;
double g_pitch;
double g_yaw;
bool g_filter;
int g_cost_base;

pcl::PointCloud<pcl::PointXYZ> g_obstacle_sim_points;
bool g_use_obstacle_sim = false;

void callbackFromObstacleSim(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::fromROSMsg(*msg, g_obstacle_sim_points);

  g_use_obstacle_sim = true;
}

void joinPoints(const pcl::PointCloud<pcl::PointXYZ> &points1, pcl::PointCloud<pcl::PointXYZ> *points2)
{
  for (const auto &p : points1)
  {
    points2->push_back(p);
  }
}

std::vector<int> createCostMap(const pcl::PointCloud<pcl::PointXYZ> &scan)
{
  std::vector<int> cost_map(g_cell_width * g_cell_height, 0);
  double map_center_x = g_map_width / 2.0 - g_offset_x;
  double map_center_y = g_map_height / 2.0 - g_offset_y;
  for (int r = 0; r < g_cell_height; r++)
  {
    for (int c = 0; c < g_cell_width; c++)
    {
      if (r < 2 || r > g_cell_height - 2)
      {
        cost_map[r * g_cell_width + c] = -1;
      }
      else if (c < 2 || c > g_cell_width - 2)
      {
        cost_map[r * g_cell_width + c] = -1;
      }
    }
  }

  // scan points are in sensor frame
  for (const auto &p : scan.points)
  {
    if (p.z > HEIGHT_LIMIT)
    {
      // ROS_INFO("x: %.2f, y: %.2f, z: %.2f", p.x, p.y, p.z);
      continue;
    }
    if (std::fabs(p.x) < CAR_LENGTH && std::fabs(p.y) < CAR_WIDTH)
      continue;

    // Calculate grid index
    int grid_x = (p.x + map_center_x) / g_resolution;
    int grid_y = (p.y + map_center_y) / g_resolution;
    if (grid_x < 0 || grid_x >= g_cell_width || grid_y < 0 || grid_y >= g_cell_height)
    {
      // ROS_INFO("x: %.2f, y: %.2f", p.x, p.y);
      continue;
    }

    int index = g_cell_width * grid_y + grid_x;
    cost_map[index] += g_cost_base;

    // Max cost value is 100
    if (cost_map[index] > 100)
      cost_map[index] = 100;
  }

  return cost_map;
}

void setOccupancyGrid(nav_msgs::OccupancyGrid *og)
{
  tf::Quaternion tmp_q;
  tmp_q.setRPY(g_roll, g_pitch, g_yaw);
  og->info.resolution = g_resolution;
  og->info.width = g_cell_width;
  og->info.height = g_cell_height;
  og->info.origin.position.x = (-1) * g_map_width / 2.0 + g_offset_x;
  og->info.origin.position.y = (-1) * g_map_height / 2.0 + g_offset_y;
  og->info.origin.position.z = g_offset_z;
  og->info.origin.orientation.x = tmp_q.getX();
  og->info.origin.orientation.y = tmp_q.getY();
  og->info.origin.orientation.z = tmp_q.getZ();
  og->info.origin.orientation.w = tmp_q.getW();
}

std::vector<int> filterCostMap(std::vector<int> &cost_map)
{
  std::vector<int> filtered_cost_map(cost_map.size(), 0);

  // cells around reference (x, y)
  std::vector<std::pair<int, int>> neighborhood{
      std::make_pair(-1, -1),
      std::make_pair(0, -1),
      std::make_pair(1, -1),
      std::make_pair(-1, 0),
      std::make_pair(1, 0),
      std::make_pair(-1, 1),
      std::make_pair(0, 1),
      std::make_pair(1, 1),
  };

  for (size_t size = cost_map.size(), i = 0; i < size; i++)
  {
    int ref_cost = cost_map[i];

    int ref_x = i % g_cell_width;
    int ref_y = (i - ref_x) / g_cell_width;

    // we don't have to filter if the cost is 0
    if (ref_cost <= 0)
    {
      filtered_cost_map[i] = ref_cost;
      continue;
    }

    filtered_cost_map[i] += ref_cost;

    // increase the cost for each neighborhood cell
    for (const auto &n : neighborhood)
    {
      int neighbor_x = ref_x + n.first;
      int neighbor_y = ref_y + n.second;

      if (neighbor_x < 0 || neighbor_x >= g_cell_width || neighbor_y < 0 || neighbor_y >= g_cell_height)
        continue;

      int neighbor_index = neighbor_x + neighbor_y * g_cell_width;
      filtered_cost_map[neighbor_index] += ref_cost;
    }
  }

  // handle the cost over 100
  for (auto &cost : filtered_cost_map)
  {
    if (cost > 100)
      cost = 100;
  }

  return filtered_cost_map;
}

void createOccupancyGrid(const sensor_msgs::PointCloud2::ConstPtr &input)
{
  static int count = 0;
  pcl::PointCloud<pcl::PointXYZ> scan;
  pcl::fromROSMsg(*input, scan);

  // use simulated obstacle
  if (g_use_obstacle_sim)
  {
    joinPoints(g_obstacle_sim_points, &scan);
    g_obstacle_sim_points.clear();
  }
  // ---

  static nav_msgs::OccupancyGrid og;
  if (count % 2 == 0)
    setOccupancyGrid(&og);

  og.header = input->header;

  // create cost map with pointcloud
  std::vector<int> cost_map = createCostMap(scan);

  if (g_filter)
  {
    cost_map = filterCostMap(cost_map);
  }

  og.data.insert(og.data.end(), cost_map.begin(), cost_map.end());
  g_costmap_pub.publish(og);
  og.data.clear();
  count++;
}

void cfgCB(const obj_map::Points2CostmapConfig &config, uint32_t level)
{
  g_cost_base = config.costBase;
  g_offset_x = config.offset_x;
  g_offset_y = config.offset_y;
  g_offset_z = config.offset_z;
  g_roll = config.roll;
  g_pitch = config.pitch;
  g_yaw = config.yaw;
  g_map_width = config.map_width;
  g_map_height = config.map_height;
  g_cell_width = static_cast<int>(std::ceil(g_map_width / g_resolution));
  g_cell_height = static_cast<int>(std::ceil(g_map_height / g_resolution));
}

} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points2costmap");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Subscribing topic of PointCloud2 message
  std::string points_topic;

  private_nh.param<double>("resolution", g_resolution, 0.1);
  private_nh.param<double>("map_width", g_map_width, 50.);
  private_nh.param<double>("map_height", g_map_height, 50.);
  private_nh.param<std::string>("points_topic", points_topic, "points_lanes");
  private_nh.param<double>("offset_x", g_offset_x, 30.0);
  private_nh.param<double>("offset_y", g_offset_y, 0.0);
  private_nh.param<double>("offset_z", g_offset_z, -2.0);
  private_nh.param<double>("height_limit", HEIGHT_LIMIT, 0.1);
  private_nh.param<double>("car_width", CAR_WIDTH, 1.75);
  private_nh.param<double>("car_length", CAR_LENGTH, 4.5);
  private_nh.param<bool>("filter", g_filter, false);
  private_nh.param<int>("cost_base", g_cost_base, 15);
  g_cell_width = static_cast<int>(std::ceil(g_map_width / g_resolution));
  g_cell_height = static_cast<int>(std::ceil(g_map_height / g_resolution));

  g_costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("realtime_cost_map", 10);
  ros::Subscriber points_sub = nh.subscribe(points_topic, 10, createOccupancyGrid);
  ros::Subscriber obstacle_sim_points_sub = nh.subscribe("obstacle_sim_pointcloud", 1, callbackFromObstacleSim);
  dynamic_reconfigure::Server<obj_map::Points2CostmapConfig> cfg_server;
  dynamic_reconfigure::Server<obj_map::Points2CostmapConfig>::CallbackType cfg_callback = boost::bind(&cfgCB, _1, _2);
  cfg_server.setCallback(cfg_callback);

  ros::spin();
}

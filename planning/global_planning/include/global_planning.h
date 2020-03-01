/*
 * @Author: fengk 
 * @Date: 2019-04-03 19:36:14 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-03 21:09:15
 */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <adam_msgs/Road.h>
#define INF 999999
#define distance2points(from, to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
#define distance2pointsSqr(from, to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define pointNorm(v) sqrt(v.x *v.x + v.y * v.y)
#define angle2points(from, to) atan2(to.y - from.y, to.x - from.x)

class GlobalPlanning
{
public:
  std::vector<std::vector<double> > net;
  std::vector<bool> visit;
  std::vector<double> longs;
  std::vector<int> pre;
  std::vector<int> pos;

  adam_msgs::Road road;
  int n_road;

  bool bInitPos;
  bool bGoalPos;
  bool bMap;
  adam_msgs::WayPoint m_CurrentPos;
  adam_msgs::WayPoint m_InitPos;
  adam_msgs::WayPoint m_GoalPos;
  nav_msgs::Path global_path;

  ros::NodeHandle nh_,pnh_;

  double smoothingDataWeight;
	double smoothingSmoothWeight;
	double smoothingToleranceError;
  double pathDensity;

  ros::Publisher pub_globalTrajectories;
  ros::Publisher pub_globalTrajectoriesrviz;

  ros::Subscriber sub_initial_pose;
  ros::Subscriber sub_current_pose;
  ros::Subscriber sub_goal_pose;
  ros::Subscriber sub_road;

  void callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &input);
  void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg);
  void callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg);
  void callbackGetRoad(const adam_msgs::RoadConstPtr &msg);

  void InitalNet(int start, int start_i, int goal, int goal_i);

  void Dijkstra(int s);
  
  void GlobalTrajectoriesToMarker();
  nav_msgs::Path GenGlobalTrajectory(int start,int goal);
  void DFS(int s, int v);  
  int GetClosestNextPointIndex(const adam_msgs::Road &trajs, const adam_msgs::WayPoint &p,int &index);
  void SmoothPath(nav_msgs::Path &path, double weight_data,double weight_smooth, double tolerance);
  void FixPathDensity(nav_msgs::Path &path, const double &distanceDensity);

  GlobalPlanning();
  ~GlobalPlanning();
  void MainLoop();
};



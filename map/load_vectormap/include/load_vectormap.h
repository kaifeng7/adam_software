/*
 * @Author: fengk 
 * @Date: 2019-04-03 14:23:23 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-10 15:37:02
 */
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <adam_msgs/Road.h>
#include <fstream>
#include <sstream>
#include "map.pb.h"
#include "load_vectormap_util.h"
#define distance2pointsSqr(from, to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)



class Header
{
public:
  //    uint32 rev_major = 1; // uShort in spec file
  //    uint32 rev_minor = 2; // uShort in spec file
  float version;
  std::string name;
  std::string date;
  //    double north = 4;
  //    double south = 5;
  //    double east = 6;
  //    double west = 7;
};

class Geometry
{
public:
  double s;
  double x;
  double y;
  double hdg;
  double length;
  enum SHAPE
  {
    Line,
    Arc
    //         Spiral spiral = 8;
    //         Poly3 poly3 = 9;
    //         ParamPoly3 paramPoly3 = 10;
  } shape;

  double curvature;
};

//message Spiral {}

//message Poly3 {}

//message ParamPoly3 {}

class RoadLinker
{
public:
  std::string element_type;
  std::string element_id;
  std::string contact_point;
};

class RoadLink
{
public:
  RoadLinker predecessor;
  RoadLinker successor;
};

class PlainView
{
public:
  std::vector<Geometry> geometry;
};

//message ElevationProfile {}

//message LateralProfile {}

class LaneWidth
{
public:
  double s_offset;
  double a;
  double b;
  double c;
  double d;
};

class RoadMark
{
public:
  //    double s_offset = 1;
  std::string type;
  //    string weight = 3;
  std::string color;
  //    string material = 5;
  double width;
  //    string lane_change = 7;
  //    double height = 8;
};

class Lane
{
public:
  int id;
  //    string type = 2;
  //    string level = 3;
  LaneWidth width;
  std::vector<RoadMark> road_mark;
};

class LaneSet
{
public:
  std::vector<Lane> lane;
};

class LaneSection
{
public:
  double s;
  LaneSet left;
  LaneSet center;
  LaneSet right;
};

class Lanes
{
public:
  LaneSection lane_section;
};
//message Object {}

//message Signal {}

class Road
{
public:
  std::string name;
  //    double length = 2;
  std::string id;
  //    string junction = 4;
  RoadLink link;
  PlainView plain_view;
  //    ElevationProfile elevation_profile = 7;
  //    LateralProfile lateral_profile = 8;
  std::vector<Lanes> lanes;
  //    repeated Object object = 10;
  //    repeated Signal signal = 11;
};

class A
{
};
class B
{
  std::string name;
};

class ConnectionLaneLink
{
  public:
    int from;
    int to;
};

class Connection
{
  public:
    std::string id;
    std::string incoming_road;
    std::string connecting_road;
    std::string contact_point;
    std::vector<ConnectionLaneLink> lane_link;

};

class Junction
{
  public:
  std::string name;
  std::string id;
  std::vector<Connection> connection;
};

class Map
{
public:
  Header header;
  std::vector<Road> road;
  std::vector<Junction> junction;
};

class LoadMap
{
public:
  LoadMap();
  virtual ~LoadMap();

  void MainLoop();
  int ConvertMaptoAllPlainView(std::vector<std::vector<adam_msgs::WayPoint> > &m_AllPlainViews);
  int ConvertMaptoAllLanes(std::vector<std::vector<std::vector<std::vector<adam_msgs::WayPoint> > > > &m_AllLanes,adam_msgs::Road &trajs);
  int ConvertMaptoAllJunctions(adam_msgs::Road &trajs);

  int ConvertpbtoMap(adam::map::Map pb_map);

  std::vector<adam_msgs::WayPoint> ConvertGeometrytoPath(Geometry m_Geometry);
  visualization_msgs::MarkerArray TrajToMarkers(adam_msgs::Road all_paths);
  visualization_msgs::MarkerArray PathToMarkers(const std::vector<std::vector<adam_msgs::WayPoint> > all_paths);
  visualization_msgs::MarkerArray LaneToMarkers(std::vector<std::vector<std::vector<std::vector<adam_msgs::WayPoint> > > > all_paths);
  visualization_msgs::MarkerArray StartToMarkers(std::vector<std::vector<adam_msgs::WayPoint> > all_paths);
  visualization_msgs::MarkerArray NumberToMarkers(adam_msgs::Road trajs);

  double FixNegativeAngle(const double& a);



  Map map;
  std::string param_map_file;
  adam_msgs::Road trajs;
  std::vector<std::vector<adam_msgs::WayPoint> > m_AllPlainViews;
  std::vector<std::vector<std::vector<std::vector<adam_msgs::WayPoint> > > >m_AllLanes;

  ros::NodeHandle nh_,pnh_;
  //define publishers
  ros::Publisher pub_AllPaths;
  ros::Publisher pub_AllPathsRviz;
  ros::Publisher pub_AllStartsRviz;
  ros::Publisher pub_AllLanesRviz;
  ros::Publisher pub_AllTrajRviz;
  ros::Publisher pub_AllNumberRviz;

};

/*
 * @Author: fengk 
 * @Date: 2019-04-03 16:45:06 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-10 15:39:36
 */
#include "load_vectormap.h"

LoadMap::LoadMap() : pnh_("~")
{
  pnh_.param<std::string>("vectormap", param_map_file, "src/adam_software/map_0725/load_vectormap/map/zhiquan_v2");
  pub_AllPaths = nh_.advertise<adam_msgs::Road>("All_Paths", 1);
  pub_AllPathsRviz = nh_.advertise<visualization_msgs::MarkerArray>("All_Paths_Rviz", 1);
  pub_AllStartsRviz = nh_.advertise<visualization_msgs::MarkerArray>("All_Starts_Rviz", 1);
  pub_AllLanesRviz = nh_.advertise<visualization_msgs::MarkerArray>("All_Lanes_Rviz", 1);
  pub_AllTrajRviz = nh_.advertise<visualization_msgs::MarkerArray>("All_traj_Rviz", 1);
  pub_AllNumberRviz = nh_.advertise<visualization_msgs::MarkerArray>("All_number_Rviz", 1);
}

LoadMap::~LoadMap()
{
}

int LoadMap::ConvertpbtoMap(adam::map::Map pb_map)
{
  Header header;
  header.version = pb_map.header().version();
  header.name = pb_map.header().name();
  header.date = pb_map.header().date();

  map.header = header;

  for (int i = 0; i < pb_map.road_size(); i++)
  {
    Road road;
    road.name = pb_map.road(i).name();
    road.id = pb_map.road(i).id();

    RoadLink link;

    RoadLinker linker_predecessor;
    linker_predecessor.element_type = pb_map.road(i).link().predecessor().element_type();
    linker_predecessor.element_id = pb_map.road(i).link().predecessor().element_id();
    linker_predecessor.contact_point = pb_map.road(i).link().predecessor().contact_point();
    link.predecessor = linker_predecessor;

    RoadLinker linker_successor;
    linker_successor.element_type = pb_map.road(i).link().successor().element_type();
    linker_successor.element_id = pb_map.road(i).link().successor().element_id();
    linker_successor.contact_point = pb_map.road(i).link().successor().contact_point();
    link.successor = linker_successor;

    road.link = link;
    PlainView plain_view;

    for (int j = 0; j < pb_map.road(i).plain_view().geometry_size(); j++)
    {
      Geometry geometry;
      geometry.s = pb_map.road(i).plain_view().geometry(j).s();
      geometry.x = pb_map.road(i).plain_view().geometry(j).x();
      geometry.y = pb_map.road(i).plain_view().geometry(j).y();
      geometry.hdg = pb_map.road(i).plain_view().geometry(j).hdg();
      geometry.length = pb_map.road(i).plain_view().geometry(j).length();

      if (pb_map.road(i).plain_view().geometry(j).shape_case() == 6)
      {
        geometry.shape = Geometry::SHAPE::Line;
      }
      else if (pb_map.road(i).plain_view().geometry(j).shape_case() == 7)
      {
        geometry.shape = Geometry::SHAPE::Arc;
        geometry.curvature = pb_map.road(i).plain_view().geometry(j).arc().curvature();
      }
      plain_view.geometry.push_back(geometry);
    }
    road.plain_view = plain_view;

    for (int j = 0; j < pb_map.road(i).lanes_size(); j++)
    {
      Lanes lanes;

      LaneSection lane_section;

      lane_section.s = pb_map.road(i).lanes(j).lane_section().s();

      LaneSet lane_set_left;
      for (int k = 0; k < pb_map.road(i).lanes(j).lane_section().left().lane_size(); k++)
      {
        Lane lane;
        lane.id = pb_map.road(i).lanes(j).lane_section().left().lane(k).id();

        LaneWidth width;
        width.s_offset = pb_map.road(i).lanes(j).lane_section().left().lane(k).width().s_offset();
        width.a = pb_map.road(i).lanes(j).lane_section().left().lane(k).width().a();
        width.b = pb_map.road(i).lanes(j).lane_section().left().lane(k).width().b();
        width.c = pb_map.road(i).lanes(j).lane_section().left().lane(k).width().c();
        width.d = pb_map.road(i).lanes(j).lane_section().left().lane(k).width().d();
        lane.width = width;

        for (int l = 0; l < pb_map.road(i).lanes(j).lane_section().left().lane(k).road_mark_size(); l++)
        {
          RoadMark road_mark;
          road_mark.type = pb_map.road(i).lanes(j).lane_section().left().lane(k).road_mark(l).type();
          road_mark.color = pb_map.road(i).lanes(j).lane_section().left().lane(k).road_mark(l).color();
          road_mark.width = pb_map.road(i).lanes(j).lane_section().left().lane(k).road_mark(l).width();
          lane.road_mark.push_back(road_mark);
        }
        lane_set_left.lane.push_back(lane);
      }
      lane_section.left = lane_set_left;

      LaneSet lane_set_center;
      for (int k = 0; k < pb_map.road(i).lanes(j).lane_section().center().lane_size(); k++)
      {
        Lane lane;
        lane.id = pb_map.road(i).lanes(j).lane_section().center().lane(k).id();

        LaneWidth width;
        width.s_offset = pb_map.road(i).lanes(j).lane_section().center().lane(k).width().s_offset();
        width.a = pb_map.road(i).lanes(j).lane_section().center().lane(k).width().a();
        width.b = pb_map.road(i).lanes(j).lane_section().center().lane(k).width().b();
        width.c = pb_map.road(i).lanes(j).lane_section().center().lane(k).width().c();
        width.d = pb_map.road(i).lanes(j).lane_section().center().lane(k).width().d();
        lane.width = width;

        for (int l = 0; l < pb_map.road(i).lanes(j).lane_section().center().lane(k).road_mark_size(); l++)
        {
          RoadMark road_mark;
          road_mark.type = pb_map.road(i).lanes(j).lane_section().center().lane(k).road_mark(l).type();
          road_mark.color = pb_map.road(i).lanes(j).lane_section().center().lane(k).road_mark(l).color();
          road_mark.width = pb_map.road(i).lanes(j).lane_section().center().lane(k).road_mark(l).width();

          lane.road_mark.push_back(road_mark);
        }
        lane_set_center.lane.push_back(lane);
      }
      lane_section.center = lane_set_center;

      LaneSet lane_set_right;
      for (int k = 0; k < pb_map.road(i).lanes(j).lane_section().right().lane_size(); k++)
      {

        Lane lane;
        lane.id = pb_map.road(i).lanes(j).lane_section().right().lane(k).id();

        LaneWidth width;
        width.s_offset = pb_map.road(i).lanes(j).lane_section().right().lane(k).width().s_offset();
        width.a = pb_map.road(i).lanes(j).lane_section().right().lane(k).width().a();
        width.b = pb_map.road(i).lanes(j).lane_section().right().lane(k).width().b();
        width.c = pb_map.road(i).lanes(j).lane_section().right().lane(k).width().c();
        width.d = pb_map.road(i).lanes(j).lane_section().right().lane(k).width().d();
        lane.width = width;

        for (int l = 0; l < pb_map.road(i).lanes(j).lane_section().right().lane(k).road_mark_size(); l++)
        {
          RoadMark road_mark;
          road_mark.type = pb_map.road(i).lanes(j).lane_section().right().lane(k).road_mark(l).type();
          road_mark.color = pb_map.road(i).lanes(j).lane_section().right().lane(k).road_mark(l).color();
          road_mark.width = pb_map.road(i).lanes(j).lane_section().right().lane(k).road_mark(l).width();

          lane.road_mark.push_back(road_mark);
        }
        lane_set_right.lane.push_back(lane);
      }
      lane_section.right = lane_set_right;

      lanes.lane_section = lane_section;

      road.lanes.push_back(lanes);
    }

    map.road.push_back(road);
  }

  for (int i = 0; i < pb_map.junction_size(); i++)
  {
    Junction junction;
    junction.name = pb_map.junction(i).name();
    junction.id = pb_map.junction(i).id();
    for (int j = 0; j < pb_map.junction(i).connection_size(); j++)
    {
      Connection connection;
      connection.id = pb_map.junction(i).connection(j).id();
      connection.incoming_road = pb_map.junction(i).connection(j).incoming_road();
      connection.connecting_road = pb_map.junction(i).connection(j).connecting_road();
      connection.contact_point = pb_map.junction(i).connection(j).contact_point();
      for (int k = 0; k < pb_map.junction(i).connection(j).lane_link_size(); k++)
      {
        ConnectionLaneLink connection_lank_link;
        connection_lank_link.from = pb_map.junction(i).connection(j).lane_link(k).from();
        connection_lank_link.to = pb_map.junction(i).connection(j).lane_link(k).to();
        connection.lane_link.push_back(connection_lank_link);
      }
      junction.connection.push_back(connection);
    }

    map.junction.push_back(junction);
  }

  google::protobuf::ShutdownProtobufLibrary();
}

int LoadMap::ConvertMaptoAllPlainView(std::vector<std::vector<adam_msgs::WayPoint>> &m_AllPlainViews)
{

  for (int i = 0; i < map.road.size(); i++)
  {
    std::vector<adam_msgs::WayPoint> m_Path;
    std::istringstream road_id(map.road.at(i).id);
    char c;
    int id;
    road_id >> c >> c >> c >> c >> id;

    for (int j = 0; j < map.road.at(i).plain_view.geometry.size(); j++)
    {
      Geometry m_Geometry = map.road.at(i).plain_view.geometry.at(j);
      adam_msgs::WayPoint wp;
      if (m_Geometry.shape == Geometry::SHAPE::Line)
      {
        for (double step = 0.0; step < m_Geometry.length; step += 0.1)
        {

          wp.ps.pose.position.x = m_Geometry.x + step * cos(m_Geometry.hdg);
          wp.ps.pose.position.y = m_Geometry.y + step * sin(m_Geometry.hdg);

          wp.road_id = id;
          wp.cost = m_Geometry.s + step;
          wp.angle = m_Geometry.hdg;
          wp.curvature = 0;
          m_Path.push_back(wp);
        }
        wp.ps.pose.position.x = m_Geometry.x + m_Geometry.length * cos(m_Geometry.hdg);
        wp.ps.pose.position.y = m_Geometry.y + m_Geometry.length * sin(m_Geometry.hdg);
        wp.road_id = id;
        wp.cost = m_Geometry.s + m_Geometry.length;
        wp.angle = m_Geometry.hdg;
        wp.curvature = 0;

        m_Path.push_back(wp);
      }
      else if (m_Geometry.shape == Geometry::SHAPE::Arc)
      {
        double a, alpha;
        double last_x = m_Geometry.x;
        double last_y = m_Geometry.y;
        wp.ps.pose.position.x = last_x;
        wp.ps.pose.position.y = last_y;
        wp.road_id = id;
        wp.cost = m_Geometry.s;
        wp.angle = m_Geometry.hdg;
        wp.curvature = m_Geometry.curvature;
        // m_Path.push_back(wp);不将第一个点加入路径
        for (double step = 0.1; step < m_Geometry.length; step += 0.1)
        {

          if (m_Geometry.curvature > 0)
          {
            a = 2 / m_Geometry.curvature * sin(step * m_Geometry.curvature / 2);
            alpha = (M_PI - step * m_Geometry.curvature) / 2 - (m_Geometry.hdg - M_PI_2);
          }
          else
          {
            a = 2 / m_Geometry.curvature * sin(step * m_Geometry.curvature / 2);
            alpha = (M_PI - step * m_Geometry.curvature) / 2 - (m_Geometry.hdg + M_PI_2);
          }
          wp.ps.pose.position.x = m_Geometry.x - 1 * a * cos(alpha);
          wp.ps.pose.position.y = m_Geometry.y + a * sin(alpha);
          wp.road_id = id;
          wp.cost = m_Geometry.s + step;
          wp.angle = FixNegativeAngle(atan2(wp.ps.pose.position.y - last_y, wp.ps.pose.position.x - last_x));
          wp.curvature = m_Geometry.curvature;
          last_y = wp.ps.pose.position.y;
          last_x = wp.ps.pose.position.x;

          m_Path.push_back(wp);
        }
        if (m_Geometry.curvature > 0)
        {
          a = 2 / m_Geometry.curvature * sin(m_Geometry.length * m_Geometry.curvature / 2);
          alpha = (M_PI - m_Geometry.length * m_Geometry.curvature) / 2 - (m_Geometry.hdg - M_PI_2);
        }
        else
        {
          a = 2 / m_Geometry.curvature * sin(m_Geometry.length * m_Geometry.curvature / 2);
          alpha = (M_PI - m_Geometry.length * m_Geometry.curvature) / 2 - (m_Geometry.hdg + M_PI_2);
        }
        wp.ps.pose.position.x = m_Geometry.x - 1 * a * cos(alpha);
        wp.ps.pose.position.y = m_Geometry.y + a * sin(alpha);
        wp.road_id = id;
        wp.cost = m_Geometry.s + m_Geometry.length;

        wp.angle = FixNegativeAngle(atan2(wp.ps.pose.position.y - last_y, wp.ps.pose.position.x - last_x));
        wp.curvature = m_Geometry.curvature;

        m_Path.push_back(wp);
      }
      // if(m_Path.size()>1)
      //     m_Path.at(0).angle = m_Path.at(1).angle;
    }
    m_AllPlainViews.push_back(m_Path);
  }
}

int LoadMap::ConvertMaptoAllLanes(std::vector<std::vector<std::vector<std::vector<adam_msgs::WayPoint>>>> &m_AllLanes, adam_msgs::Road &trajs)
{
  for (int i = 0; i < map.road.size(); i++)
  { //初始化
    trajs.lanes.push_back(adam_msgs::Lane());
  }

  for (int i = 0; i < map.road.size(); i++)
  {
    Road road = map.road.at(i);
    adam_msgs::Lane traj;
    traj.s = 0;
    std::vector<std::vector<std::vector<adam_msgs::WayPoint>>> m_Lanes;
    for (int j = 0; j < road.lanes.size(); j++)
    {
      std::vector<std::vector<adam_msgs::WayPoint>> LaneSections;
      LaneSection lane_section = road.lanes.at(j).lane_section;
      LaneSection lane_next_section;
      // if (j < road.lanes.size() - 1)
      //   lane_next_section = road.lanes.at(j + 1).lane_section;
      // else
      //   lane_next_section = road.lanes.at(j).lane_section;

      LaneSet lane_left = lane_section.left;
      for (int k = 0; k < lane_left.lane.size(); k++)
      {
        double width = lane_left.lane.at(k).width.a;
        std::vector<adam_msgs::WayPoint> Lanes;
        for (int l = 0; l < m_AllPlainViews.at(i).size(); l++)
        {
          adam_msgs::WayPoint wp = m_AllPlainViews.at(i).at(l);
          // if (wp.cost < lane_section.s)
          //   continue;
          // if (wp.cost == lane_next_section.s)
          //   break;

          Mat3 transform(width * cos(wp.angle - M_PI_2), width * sin(wp.angle - M_PI_2));
          adam_msgs::WayPoint wp_lane;
          wp_lane = transform * wp;
          wp_lane.angle = wp.angle;
          wp_lane.curvature = wp.curvature;
          if (wp_lane.cost > 0)
            wp_lane.cost = wp.cost + width * wp.curvature;
          wp_lane.road_id = wp.road_id;
          wp_lane.lane_id = lane_left.lane.at(k).id;
          adam_msgs::WayPoint wp_traj;

          wp_traj.ps.pose.position.x = (wp.ps.pose.position.x + wp_lane.ps.pose.position.x) / 2;
          wp_traj.ps.pose.position.y = (wp.ps.pose.position.y + wp_lane.ps.pose.position.y) / 2;
          wp_traj.ps.pose.position.z = (wp.ps.pose.position.z + wp_lane.ps.pose.position.z) / 2;
          wp_traj.cost = (wp.cost + wp_lane.cost) / 2;
          wp_traj.road_id = wp.road_id;
          wp_traj.lane_id = wp.lane_id;
          wp_traj.curvature = wp.curvature;
          wp_traj.angle = wp.angle;
          traj.wps.push_back(wp_traj);

          traj.s = wp.cost;
          Lanes.push_back(wp_lane);
        }
        std::istringstream road_id(road.id);
        char c;
        int id;
        road_id >> c >> c >> c >> c >> id;
        traj.id = id;
        trajs.lanes.at(id) = traj;
        LaneSections.push_back(Lanes);
      }
      LaneSet lane_right = lane_section.right;
      for (int k = 0; k < lane_right.lane.size(); k++)
      {
        double width = lane_right.lane.at(k).width.a;
        std::vector<adam_msgs::WayPoint> Lanes;
        for (int l = 0; l < m_AllPlainViews.at(i).size(); l++)
        {
          adam_msgs::WayPoint wp = m_AllPlainViews.at(i).at(l);
          // if (wp.cost < lane_section.s)
          //   continue;
          // if (wp.cost == lane_next_section.s)
          //   break;

          Mat3 transform(width * cos(wp.angle + M_PI_2), width * sin(wp.angle + M_PI_2));
          adam_msgs::WayPoint wp_lane;
          wp_lane = transform * wp;
          wp_lane.angle = wp.angle;
          wp_lane.curvature = wp.curvature;

          if (wp_lane.cost > 0)
            wp_lane.cost = wp.cost + width * wp.curvature;
          wp_lane.road_id = wp.road_id;
          wp_lane.lane_id = lane_right.lane.at(k).id;
          adam_msgs::WayPoint wp_traj;

          wp_traj.ps.pose.position.x = (wp.ps.pose.position.x + wp_lane.ps.pose.position.x) / 2;
          wp_traj.ps.pose.position.y = (wp.ps.pose.position.y + wp_lane.ps.pose.position.y) / 2;
          wp_traj.ps.pose.position.z = (wp.ps.pose.position.z + wp_lane.ps.pose.position.z) / 2;
          wp_traj.cost = (wp.cost + wp_lane.cost) / 2;
          wp_traj.road_id = wp.road_id;
          wp_traj.lane_id = wp.lane_id;
          wp_traj.curvature = wp.curvature;
          wp_traj.angle = wp.angle;
          traj.wps.push_back(wp_traj);

          traj.s = wp.cost;
          Lanes.push_back(wp_lane);
        }
        std::istringstream road_id(road.id);
        char c;
        int id;
        road_id >> c >> c >> c >> c >> id;
        traj.id = id;
        trajs.lanes.at(id) = traj;
        LaneSections.push_back(Lanes);
      }

      m_Lanes.push_back(LaneSections);
    }
    m_AllLanes.push_back(m_Lanes);
  }
}

int LoadMap::ConvertMaptoAllJunctions(adam_msgs::Road &trajs)
{
  for (int i = 0; i < map.junction.size(); i++)
  { //连接点为juction的路
    Junction junction = map.junction.at(i);
    for (int j = 0; j < junction.connection.size(); j++)
    {
      Connection connection = junction.connection.at(j);
      std::istringstream in(connection.incoming_road);
      std::istringstream out(connection.connecting_road);
      int in_num, out_num;
      char c;
      in >> c >> c >> c >> c >> in_num;
      out >> c >> c >> c >> c >> out_num;

      trajs.lanes.at(in_num).next_id.push_back(out_num);
      trajs.lanes.at(out_num).pre_id.push_back(in_num);
    }
  }
  for (int i = 0; i < map.road.size(); i++)
  { //连接点为road的路
    Road road = map.road.at(i);
    RoadLinker predecessor = road.link.predecessor;
    RoadLinker successor = road.link.successor;
    if (predecessor.element_type == "road")
    {
      std::istringstream in(predecessor.element_id);
      std::istringstream out(road.id);
      int in_num, out_num;
      char c;
      in >> c >> c >> c >> c >> in_num;
      out >> c >> c >> c >> c >> out_num;
      trajs.lanes.at(in_num).next_id.push_back(out_num);
      trajs.lanes.at(out_num).pre_id.push_back(in_num);

      trajs.lanes.at(in_num).pre_id.push_back(out_num);
      trajs.lanes.at(out_num).next_id.push_back(in_num);
    }

    if (successor.element_type == "road")
    {
      std::istringstream in(road.id);

      std::istringstream out(successor.element_id);
      int in_num, out_num;
      char c;
      in >> c >> c >> c >> c >> in_num;
      out >> c >> c >> c >> c >> out_num;
      trajs.lanes.at(out_num).pre_id.push_back(in_num);
      trajs.lanes.at(in_num).next_id.push_back(out_num);
      
      trajs.lanes.at(out_num).next_id.push_back(in_num);
      trajs.lanes.at(in_num).pre_id.push_back(out_num);
    }
  }
}

double LoadMap::FixNegativeAngle(const double& a)
{
   double angle = 0;
   if (a < -2.0*M_PI || a > 2.0*M_PI)
	{
	   angle = fmod(a, 2.0*M_PI);
	}
   else
	   angle = a;


   if(angle < 0)
   {
	   angle = 2.0*M_PI + angle;
   }

   return angle;
}


void LoadMap::MainLoop()
{
  std::fstream input(param_map_file, std::ios::in | std::ios::binary);
  if (!input.is_open())
  {
    std::cout << "fail to open map\n";
    exit(-1);
  }
  adam::map::Map pb_map;
  pb_map.ParseFromIstream(&input);
  ConvertpbtoMap(pb_map);
  ConvertMaptoAllPlainView(m_AllPlainViews);
  ConvertMaptoAllLanes(m_AllLanes, trajs);
  ConvertMaptoAllJunctions(trajs);

  ROS_INFO_STREAM("vector map server start");
  ROS_INFO_STREAM("map_path:" << param_map_file);

  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    ros::spinOnce();

    visualization_msgs::MarkerArray all_path_markers, all_start_markers, all_lane_markers, all_traj_markers, all_number_markers;
    all_path_markers = PathToMarkers(m_AllPlainViews);
    all_start_markers = StartToMarkers(m_AllPlainViews);
    all_lane_markers = LaneToMarkers(m_AllLanes);
    all_traj_markers = TrajToMarkers(trajs);
    all_number_markers = NumberToMarkers(trajs);

    pub_AllPaths.publish(trajs);
    pub_AllPathsRviz.publish(all_path_markers);
    pub_AllStartsRviz.publish(all_start_markers);
    pub_AllLanesRviz.publish(all_lane_markers);
    pub_AllTrajRviz.publish(all_traj_markers);
    pub_AllNumberRviz.publish(all_number_markers);

    loop_rate.sleep();
  }
}

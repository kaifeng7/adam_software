/*
 * @Author: fengk 
 * @Date: 2019-04-03 19:38:46 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-16 10:30:50
 */
#include "global_planning.h"


GlobalPlanning::GlobalPlanning()
    : pnh_("~"),
      bInitPos(false),
      bMap(false),
      bGoalPos(false)
{

    pub_globalTrajectories = nh_.advertise<nav_msgs::Path>("Global_trajectory", 1);
    //pub_globalTrajectoriesrviz = nh_.advertise<visualization_msgs::MarkerArray>("global_trajectory_rviz", 1);

    sub_initial_pose = nh_.subscribe("/initialpose", 1, &GlobalPlanning::callbackGetInitPose, this);
    sub_current_pose = nh_.subscribe("/current_pose", 10, &GlobalPlanning::callbackGetCurrentPose, this);
    sub_goal_pose = nh_.subscribe("/move_base_simple/goal", 1, &GlobalPlanning::callbackGetGoalPose, this);
    sub_road = nh_.subscribe("/All_Paths", 1, &GlobalPlanning::callbackGetRoad, this);

    pnh_.param<double>("smoothingDataWeight", smoothingDataWeight, 0.47);
    pnh_.param<double>("smoothingSmoothWeight", smoothingSmoothWeight, 0.2);
    pnh_.param<double>("smoothingToleranceError", smoothingToleranceError, 0.2);
    pnh_.param<double>("pathDensity", pathDensity, 0.3);
}
GlobalPlanning::~GlobalPlanning()
{
}

void GlobalPlanning::callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{

    m_InitPos.ps.pose = msg->pose.pose;
    m_InitPos.angle = tf::getYaw(msg->pose.pose.orientation);
    m_CurrentPos = m_InitPos;
    bInitPos = true;
}

void GlobalPlanning::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    m_CurrentPos.ps.pose = msg->pose;
    m_CurrentPos.angle = tf::getYaw(msg->pose.orientation);
    m_InitPos = m_CurrentPos;
    bInitPos = true;
}

void GlobalPlanning::callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    m_GoalPos.ps.pose = msg->pose;
    m_GoalPos.angle = tf::getYaw(msg->pose.orientation);
    bGoalPos = true;
}

void GlobalPlanning::callbackGetRoad(const adam_msgs::RoadConstPtr &msg)
{
    road = *msg;
    bMap = true;
}

void GlobalPlanning::InitalNet(int start, int start_i, int goal, int goal_i)
{
    n_road = road.lanes.size();
    pre.clear();
    pos.clear();
    visit.clear();
    longs.clear();
    net.clear();

    for (int i = 0; i < n_road; i++)
    {   
        std::vector<double> ds;
        for (int j = 0; j < n_road; j++)
        {
            ds.push_back(-1.0);
        }
        net.push_back(ds);
        longs.push_back(INF);
        pre.push_back(i);
        visit.push_back(false);
    }

    for (int i = 0; i < n_road; i++)
    {
        for (int j = 0; j < road.lanes.at(i).next_id.size(); j++)
        {
            int k = road.lanes.at(i).next_id.at(j);
            net.at(k).at(i) = road.lanes.at(i).s;

            if (k == start)
            {
                double d1, d2, d3, d4;
                d1 = distance2pointsSqr(road.lanes.at(i).wps.at(0).ps.pose.position, road.lanes.at(k).wps.at(0).ps.pose.position);
                d2 = distance2pointsSqr(road.lanes.at(i).wps.at(road.lanes.at(i).wps.size() - 1).ps.pose.position, road.lanes.at(k).wps.at(road.lanes.at(k).wps.size() - 1).ps.pose.position);
                d3 = distance2pointsSqr(road.lanes.at(i).wps.at(0).ps.pose.position, road.lanes.at(k).wps.at(road.lanes.at(k).wps.size() - 1).ps.pose.position);
                d4 = distance2pointsSqr(road.lanes.at(i).wps.at(road.lanes.at(i).wps.size() - 1).ps.pose.position, road.lanes.at(k).wps.at(0).ps.pose.position);
                if (d1 <= d2 && d1 <= d3 && d1 <= d4)
                    net.at(k).at(i) += road.lanes.at(k).wps.at(start_i).cost;

                else if (d2 <= d1 && d2 <= d3 && d2 <= d4)
                    net.at(k).at(i) += (road.lanes.at(k).wps.at(road.lanes.at(k).wps.size() - 1).cost - road.lanes.at(k).wps.at(start_i).cost);

                else if (d3 <= d1 && d3 <= d2 && d3 <= d4)
                    net.at(k).at(i) += (road.lanes.at(k).wps.at(road.lanes.at(k).wps.size() - 1).cost - road.lanes.at(k).wps.at(start_i).cost);

                else if (d4 <= d1 && d4 <= d2 && d4 <= d3)
                    net.at(k).at(i) += road.lanes.at(k).wps.at(start_i).cost;
            }

            if (i == goal)
            {
                double d1, d2, d3, d4;
                d1 = distance2pointsSqr(road.lanes.at(i).wps.at(0).ps.pose.position, road.lanes.at(k).wps.at(0).ps.pose.position);
                d2 = distance2pointsSqr(road.lanes.at(i).wps.at(road.lanes.at(i).wps.size() - 1).ps.pose.position, road.lanes.at(k).wps.at(road.lanes.at(k).wps.size() - 1).ps.pose.position);
                d3 = distance2pointsSqr(road.lanes.at(i).wps.at(0).ps.pose.position, road.lanes.at(k).wps.at(road.lanes.at(k).wps.size() - 1).ps.pose.position);
                d4 = distance2pointsSqr(road.lanes.at(i).wps.at(road.lanes.at(i).wps.size() - 1).ps.pose.position, road.lanes.at(k).wps.at(0).ps.pose.position);
                if (d1 <= d2 && d1 <= d3 && d1 <= d4)
                    net.at(k).at(i) -= (road.lanes.at(i).wps.at(road.lanes.at(i).wps.size() - 1).cost - road.lanes.at(i).wps.at(goal_i).cost);

                else if (d2 <= d1 && d2 <= d3 && d2 <= d4)
                    net.at(k).at(i) -= road.lanes.at(i).wps.at(goal_i).cost;

                else if (d3 <= d1 && d3 <= d2 && d3 <= d4)
                    net.at(k).at(i) -= (road.lanes.at(i).wps.at(road.lanes.at(i).wps.size() - 1).cost - road.lanes.at(i).wps.at(goal_i).cost);

                else if (d4 <= d1 && d4 <= d2 && d4 <= d3)
                    net.at(k).at(i) -= road.lanes.at(i).wps.at(goal_i).cost;
            }
        }
    }
}

void GlobalPlanning::Dijkstra(int s)
{

    longs.at(s) = 0.0;
    for (int i = 0; i < n_road; i++)
    {
        double min = INF;
        int u = -1;
        for (int j = 0; j < n_road; j++)
        {
            if (visit.at(j) == false && longs.at(j) < min)
            {
                u = j;
                min = longs.at(j);
            }
        }
        if (u == -1)
            return;
        visit[u] = true;
        for (int v = 0; v < n_road; v++)
        {
            if (visit[v] == false && net.at(u).at(v) > 0.0 && longs.at(u) + net.at(u).at(v) < longs.at(v))
            {
                longs.at(v) = longs.at(u) + net.at(u).at(v);
                pre[v] = u;
            }
        }
    }
}

void GlobalPlanning::DFS(int s, int v)
{
    if (v == s)
    {
        pos.push_back(s);
        return;
    }
    DFS(s, pre[v]);
    pos.push_back(v);
}

nav_msgs::Path GlobalPlanning::GenGlobalTrajectory(int start, int goal)
{
    nav_msgs::Path path;
    path.header.frame_id = "map";
    if (pos.size() == 0)
        return path;
     else if (pos.size() == 1)
    {
        if (start < goal)
        {
            adam_msgs::Lane lane = road.lanes.at(pos.at(0));
            path.poses.push_back(lane.wps.at(start).ps);
            for (int i = start; i < goal; i++)
            {

                geometry_msgs::PoseStamped ps;
                ps = lane.wps.at(i).ps;
                path.poses.push_back(ps);
            }
        }
        else
        {
            adam_msgs::Lane lane = road.lanes.at(pos.at(0));
            path.poses.push_back(lane.wps.at(start).ps);
            for (int i = start; i > goal; i--)
            {

                geometry_msgs::PoseStamped ps;
                ps = lane.wps.at(i).ps;
                path.poses.push_back(ps);
            }
        }
    }
    else
    {
        adam_msgs::Lane lane_first = road.lanes.at(pos.at(0));
        adam_msgs::Lane lane_second = road.lanes.at(pos.at(1));
        double d1, d2, d3, d4;
        d1 = distance2pointsSqr(lane_first.wps.at(0).ps.pose.position, lane_second.wps.at(0).ps.pose.position);
        d2 = distance2pointsSqr(lane_first.wps.at(lane_first.wps.size() - 1).ps.pose.position, lane_second.wps.at(lane_second.wps.size() - 1).ps.pose.position);
        d3 = distance2pointsSqr(lane_first.wps.at(0).ps.pose.position, lane_second.wps.at(lane_second.wps.size() - 1).ps.pose.position);
        d4 = distance2pointsSqr(lane_first.wps.at(lane_first.wps.size() - 1).ps.pose.position, lane_second.wps.at(0).ps.pose.position);
        path.poses.push_back(lane_first.wps.at(start).ps);
        if (d1 < d2 && d1 < d3 && d1 < d4)
        {
            for (int i = start; i > 0; i--)
            {
                geometry_msgs::PoseStamped ps;
                ps = lane_first.wps.at(i).ps;
                path.poses.push_back(ps);
            }
        }
        else if (d2 <= d1 && d2 <= d3 && d2 <= d4)
            for (int i = start; i < lane_first.wps.size() - 1; i++)
            {
                geometry_msgs::PoseStamped ps;
                ps = lane_first.wps.at(i).ps;
                path.poses.push_back(ps);
            }
        else if (d3 <= d1 && d3 <= d2 && d3 <= d4)
            for (int i = start; i > 0; i--)
            {
                geometry_msgs::PoseStamped ps;
                ps = lane_first.wps.at(i).ps;
                path.poses.push_back(ps);
            }
        else if (d4 <= d1 && d4 <= d2 && d4 <= d3)
            for (int i = start; i < lane_first.wps.size() - 1; i++)
            {
                geometry_msgs::PoseStamped ps;
                ps = lane_first.wps.at(i).ps;
                path.poses.push_back(ps);
            }

        for (int i = 1; i < pos.size() - 1; i++)
        {
            adam_msgs::Lane lane = road.lanes.at(pos.at(i));
            geometry_msgs::PoseStamped ps;
            double d1, d2;
            d1 = distance2pointsSqr(lane.wps.at(0).ps.pose.position, path.poses.at(path.poses.size() - 1).pose.position);
            d2 = distance2pointsSqr(lane.wps.at(lane.wps.size() - 1).ps.pose.position, path.poses.at(path.poses.size() - 1).pose.position);
            if (d1 <= d2)
            {
                for (int j = 0; j < lane.wps.size(); j++)
                {
                    ps = lane.wps.at(j).ps;
                    path.poses.push_back(ps);
                }
            }
            else
            {
                for (int j = lane.wps.size() - 1; j > 0; j--)
                {
                    ps = lane.wps.at(j).ps;
                    path.poses.push_back(ps);
                }
            }
        }
        adam_msgs::Lane lane = road.lanes.at(pos.at(pos.size() - 1));
        d1 = distance2pointsSqr(lane.wps.at(0).ps.pose.position, path.poses.at(path.poses.size() - 1).pose.position);
        d2 = distance2pointsSqr(lane.wps.at(lane.wps.size() - 1).ps.pose.position, path.poses.at(path.poses.size() - 1).pose.position);
        geometry_msgs::PoseStamped ps;
        if (d1 <= d2)
        {
            for (int j = 0; j <= goal; j++)
            {
                ps = lane.wps.at(j).ps;
                path.poses.push_back(ps);
            }
        }
        else
        {
            for (int j = lane.wps.size() - 1; j >= goal; j--)
            {
                ps = lane.wps.at(j).ps;
                path.poses.push_back(ps);
            }
        }
    }
    return path;
}

int GlobalPlanning::GetClosestNextPointIndex(const adam_msgs::Road &trajs, const adam_msgs::WayPoint &p, int &index)
{ //得到轨迹中开始距离p最近的点
    int size = (int)trajs.lanes.size();

    if (size < 2)
        return 0;

    double d = 0, minD = INF;
    int min_index = -1;
    for (int i = 0; i < size; i++)
    {
        for (unsigned int j = 0; j < trajs.lanes.at(i).wps.size(); j++)
        {
            d = distance2pointsSqr(trajs.lanes.at(i).wps.at(j).ps.pose.position, p.ps.pose.position);

            if (d < minD)
            {
                min_index = i;
                minD = d;
                index = j;
            }
        }
    }

    return min_index;
}

void GlobalPlanning::SmoothPath(nav_msgs::Path &path, double weight_data,
                                double weight_smooth, double tolerance)
{

    if (path.poses.size() <= 2)
    {
        //cout << "Can't Smooth Path, Path_in Size=" << path.size() << endl;
        return;
    }

    const nav_msgs::Path &path_in = path;
    nav_msgs::Path smoothPath_out = path_in;

    double change = tolerance;
    double xtemp, ytemp;
    int nIterations = 0;

    int size = path_in.poses.size();

    while (change >= tolerance)
    {
        change = 0.0;
        for (int i = 1; i < size - 1; i++)
        {
            //			if (smoothPath_out[i].pos.a != smoothPath_out[i - 1].pos.a)
            //				continue;

            xtemp = smoothPath_out.poses[i].pose.position.x;
            ytemp = smoothPath_out.poses[i].pose.position.y;

            smoothPath_out.poses[i].pose.position.x += weight_data * (path_in.poses[i].pose.position.x - smoothPath_out.poses[i].pose.position.x);
            smoothPath_out.poses[i].pose.position.y += weight_data * (path_in.poses[i].pose.position.y - smoothPath_out.poses[i].pose.position.y);

            smoothPath_out.poses[i].pose.position.x += weight_smooth * (smoothPath_out.poses[i - 1].pose.position.x + smoothPath_out.poses[i + 1].pose.position.x - (2.0 * smoothPath_out.poses[i].pose.position.x));
            smoothPath_out.poses[i].pose.position.y += weight_smooth * (smoothPath_out.poses[i - 1].pose.position.y + smoothPath_out.poses[i + 1].pose.position.y - (2.0 * smoothPath_out.poses[i].pose.position.y));

            change += fabs(xtemp - smoothPath_out.poses[i].pose.position.x);
            change += fabs(ytemp - smoothPath_out.poses[i].pose.position.y);
        }
        nIterations++;
    }

    path = smoothPath_out;
}

void GlobalPlanning::FixPathDensity(nav_msgs::Path &path, const double &distanceDensity)
{ //修正path中每个点的间隔
    if (path.poses.size() == 0 || distanceDensity == 0)
        return;

    double d = 0, a = 0;
    double margin = distanceDensity * 0.01;
    double remaining = 0;
    int nPoints = 0;
    nav_msgs::Path fixedPath;
    fixedPath.poses.push_back(path.poses.at(0));
    for (unsigned int si = 0, ei = 1; ei < path.poses.size();)
    {
        d += distance2points(path.poses.at(ei).pose.position, path.poses.at(ei - 1).pose.position) + remaining; //ei总距离
        a = angle2points(path.poses.at(si).pose.position, path.poses.at(ei).pose.position);                     //ei偏离si的弧度

        if (d < distanceDensity - margin) // skip
        {
            ei++;
            remaining = 0;
        }
        else if (d > (distanceDensity + margin)) // skip
        {
            geometry_msgs::PoseStamped pm = path.poses.at(si);
            nPoints = d / distanceDensity;
            for (int k = 0; k < nPoints; k++)
            {
                pm.pose.position.x = pm.pose.position.x + distanceDensity * cos(a);
                pm.pose.position.y = pm.pose.position.y + distanceDensity * sin(a);
                fixedPath.poses.push_back(pm);
            }
            remaining = d - nPoints * distanceDensity;
            si++;
            path.poses.at(si).pose.position = pm.pose.position;
            d = 0;
            ei++;
        }
        else
        {
            d = 0;
            remaining = 0;
            fixedPath.poses.push_back(path.poses.at(ei));
            ei++;
            si = ei - 1;
        }
    }

    path = fixedPath;
}

void GlobalPlanning::MainLoop()
{
    ROS_INFO_STREAM("Global_plannning start");
    ros::Rate loop_rate(5);

    nav_msgs::Path pub_path;
    while (ros::ok())
    {
        ros::spinOnce();

        if (!bInitPos)
        {
            ROS_WARN_STREAM("InitPos topic is not subscribed yet ... ");

            loop_rate.sleep();
            continue;
        }
        else if (!bMap)
        {
            ROS_WARN_STREAM("Map topic is not subscribed yet ... ");

            loop_rate.sleep();
            continue;
        }
        else if (!bGoalPos)
        {
            ROS_WARN_STREAM("GoalPos topic is not subscribed yet ... ");

            loop_rate.sleep();
            continue;
        }
        int start_i, goal_i, start_waypoint_i, goal_waypoint_i;
        start_i = GetClosestNextPointIndex(road, m_InitPos, start_waypoint_i);
        goal_i = GetClosestNextPointIndex(road, m_GoalPos, goal_waypoint_i);
        InitalNet(start_i, start_waypoint_i, goal_i, goal_waypoint_i);

        Dijkstra(start_i);
        DFS(start_i, goal_i);

        pub_path = GenGlobalTrajectory(start_waypoint_i, goal_waypoint_i);
        FixPathDensity(pub_path, pathDensity);
        SmoothPath(pub_path, smoothingDataWeight, smoothingSmoothWeight, smoothingToleranceError);
        pub_path.header.frame_id = "map";
        ROS_INFO_STREAM("Global path was planned");

        pub_globalTrajectories.publish(pub_path);
        bInitPos = false;
        bGoalPos = false;
        loop_rate.sleep();
    }
}

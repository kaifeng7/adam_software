/*
 * @Author: fengk 
 * @Date: 2019-04-02 11:19:10 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-12 17:46:53
 */

#include "local_planning.h"

TrajectoryGen::TrajectoryGen()
		: pnh_("~"),
			bInitPos(false),
			bCurrentPos(false),
			bVehicleStatus(false),
			bGlobalPath(false),
			bCostMap(false)
{
	initROS();
}

TrajectoryGen::~TrajectoryGen()
{
}

void TrajectoryGen::initROS()
{
	pnh_.param<double>("samplingTipMargin", m_PlanningParams.carTipMargin, 0);
	pnh_.param<double>("samplingOutMargin", m_PlanningParams.rollInMargin, 0);
	pnh_.param<double>("samplingSpeedFactor", m_PlanningParams.rollInSpeedFactor, 0);
	pnh_.param<double>("pathDensity", m_PlanningParams.pathDensity, 0);
	pnh_.param<double>("rollOutDensity", m_PlanningParams.rollOutDensity, 0);

	pnh_.param<int>("rollOutsNumber", m_PlanningParams.rollOutNumber, 0);

	pnh_.param<double>("rollOutDistance", m_PlanningParams.rollOutDistance, 0);

	pnh_.param<double>("planDistance", m_PlanningParams.planDistance, 0);

	pnh_.param<double>("lastInterpolationNumber", m_PlanningParams.lastInterpolationNumber, 0);

	pnh_.param<double>("smoothingDataWeight", m_PlanningParams.smoothingDataWeight, 0);
	pnh_.param<double>("smoothingSmoothWeight", m_PlanningParams.smoothingSmoothWeight, 0);
	pnh_.param<double>("smoothingToleranceError", m_PlanningParams.smoothingToleranceError, 0);

	pnh_.param<double>("laneChangeSpeedFactor", m_PlanningParams.laneChangeSpeedFactor, 0);
	pnh_.param<int>("obstacleThreshold", m_PlanningParams.obstacleThreshold, 0);

	pnh_.param<double>("WeightTransition", m_PlanningParams.weightTransition, 0);
	pnh_.param<double>("WeightPriority", m_PlanningParams.weightPriority, 0);
	pnh_.param<double>("WeightObj", m_PlanningParams.weightObj, 0);

	pnh_.param<double>("MaxSpeed", m_PlanningParams.maxSpeed, 0);
	pnh_.param<double>("MoreSpeed", m_PlanningParams.moreSpeed, 0);
	pnh_.param<double>("LessSpeed", m_PlanningParams.lessSpeed, 0);
	pnh_.param<double>("MinSpeed", m_PlanningParams.minSpeed, 0);

	pnh_.param<double>("MaxAngle", m_PlanningParams.maxAngle, 0);
	pnh_.param<double>("MidAngle", m_PlanningParams.midAngle, 0);
	pnh_.param<double>("MinAngle", m_PlanningParams.minAngle, 0);

	pub_LocalTrajectories = nh_.advertise<nav_msgs::Path>("local_trajectories", 1);
	pub_LocalTrajectoriesSpeed = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_x", 5);
	pub_LocalTrajectoriesRviz = nh_.advertise<visualization_msgs::MarkerArray>("local_planning_trajectories_rviz", 1);
	pub_LocalTrajectoriesSpeedRviz = nh_.advertise<visualization_msgs::MarkerArray>("local_planning_Speeds_rviz", 1);
	pub_LocalTrajectoriesCurrentSpeedRviz = nh_.advertise<visualization_msgs::Marker>("local_planning_Current_Speed", 5);
	pub_CarModelRviz = nh_.advertise<visualization_msgs::Marker>("car_model_rviz", 5);

	//sub_initialpose = nh_.subscribe("/initialpose", 1, &TrajectoryGen::callbackGetInitPose, this);
	sub_current_pose = nh_.subscribe("/current_pose", 10, &TrajectoryGen::callbackGetCurrentPose, this);

	//sub_robot_odom = nh_.subscribe("/odom", 10, &TrajectoryGen::callbackGetRobotOdom, this);
	//sub_current_velocity = nh_.subscribe("/current_velocity", 10, &TrajectoryGen::callbackGetVehicleStatus, this);

	sub_GlobalPlannerPaths = nh_.subscribe("/lane_waypoints_array", 1, &TrajectoryGen::callbackGetGlobalPlannerPath, this);

	sub_ObstacleMap = nh_.subscribe("/map", 10, &TrajectoryGen::callbackGetObstaclesMap, this);

	sub_CarBasicInfo = nh_.subscribe("/car_basic_info", 1, &TrajectoryGen::callbackGetCarBasicInfo, this);

	sub_CarLimitInfo = nh_.subscribe("/car_limit_info", 1, &TrajectoryGen::callbackGetCarLimitInfo, this);
}

double TrajectoryGen::CalcAngleAndCost(std::vector<adam_msgs::WayPoint> &path)
{ //计算路径中每个节点从起点的路径开销以及朝向
	if (path.size() < 2)
		return 0.;
	if (path.size() == 2) //当路径中只有两个点时
	{
		path[0].cost = 0;
		path[0].angle = utility.AngleBetweenTwoPoints(path[0].ps.pose.position, path[1].ps.pose.position);
		path[0].vel_x = m_PlanningParams.lessSpeed;

		path[1].angle = path[0].angle;
		path[1].cost = path[0].cost + utility.getPlaneDistance(path[0].ps.pose.position, path[1].ps.pose.position);
		path[1].vel_x = 0.0;
		return path[1].cost;
	}

	path[0].cost = 0;
	path[0].angle = utility.AngleBetweenTwoPoints(path[0].ps.pose.position, path[1].ps.pose.position);

	path[0].vel_x = m_PlanningParams.moreSpeed;

	for (int j = 1; j < path.size() - 1; j++)
	{
		path[j].cost = path[j - 1].cost + utility.getPlaneDistance(path[j - 1].ps.pose.position, path[j].ps.pose.position);
		path[j].angle = utility.AngleBetweenTwoPoints(path[j].ps.pose.position, path[j + 1].ps.pose.position);
		//ROS_INFO_STREAM(j<<" angle = "<<path[j].angle<<path[j].ps.pose.position.x<<" "<<path[j].ps.pose.position.y);
		double diff_angle = utility.rad2deg(utility.AngleBetweenTwoAnglesPositive(path[j].angle, path[j - 1].angle));
		if (diff_angle > m_PlanningParams.maxAngle)
			path[j - 1].vel_x = m_PlanningParams.minSpeed;
		else if (diff_angle <= m_PlanningParams.maxAngle && diff_angle > m_PlanningParams.midAngle)
			path[j - 1].vel_x = m_PlanningParams.lessSpeed;
		else if (diff_angle <= m_PlanningParams.midAngle && diff_angle > m_PlanningParams.minAngle)
			path[j - 1].vel_x = m_PlanningParams.moreSpeed;
		else if (diff_angle <= m_PlanningParams.minAngle && diff_angle >= 0)
			path[j - 1].vel_x = m_PlanningParams.maxSpeed;
	}
	int j = (int)path.size() - 1;
	path[j].vel_x = 0.0;
	path[j].angle = path[j - 1].angle;
	path[j].cost = path[j - 1].cost + utility.getPlaneDistance(path[j].ps.pose.position, path[j - 1].ps.pose.position);

	for (int i = 0; i < path.size(); i++)
	{
		if (utility.AngleBetweenTwoAnglesPositive(path[i].angle, m_CurrentPos.angle) > M_PI_2)
			path[i].vel_x = -m_PlanningParams.minSpeed;
	}
	return path[j].cost;
}

bool TrajectoryGen::CompareTrajectories(const std::vector<adam_msgs::WayPoint> &path1, const std::vector<adam_msgs::WayPoint> &path2)
{ //比较两条路径是否有区别
	if (path1.size() != path2.size())
		return false;

	for (unsigned int i = 0; i < path1.size(); i++)
	{
		if (path1.at(i).vel_x != path2.at(i).vel_x || path1.at(i).ps.pose.position.x != path2.at(i).ps.pose.position.x || path1.at(i).ps.pose.position.y != path2.at(i).ps.pose.position.y)
			return false;
	}
	return true;
}

int TrajectoryGen::GetClosestNextPointIndexDirectionFast(const std::vector<adam_msgs::WayPoint> &trajectory, const adam_msgs::WayPoint &p, const int &prevIndex)
{ //得到轨迹中从prev_index开始距离p最近的点
	int size = (int)trajectory.size();

	if (size < 2 || prevIndex < 0)
		return 0;

	double d = 0, minD = DBL_MAX;
	int min_index = prevIndex;
	for (unsigned int i = prevIndex; i < size; i++)
	{
		d = utility.getPlaneDistance2(trajectory[i].ps.pose.position, p.ps.pose.position);
		//double angle_diff = utility.rad2deg(utility.AngleBetweenTwoAnglesPositive(trajectory[i].angle, p.angle)); //夹角角度

		if (d < minD)
		{
			min_index = i;
			minD = d;
		}
	}

	if (min_index < (int)trajectory.size() - 2)
	{ //如果当前位置与最近点及下一个点的夹角大于90度,则选择下一个点为最近点
		adam_msgs::WayPoint curr, next;
		curr.ps.pose.position = trajectory.at(min_index).ps.pose.position;
		next.ps.pose.position = trajectory.at(min_index + 1).ps.pose.position;
		adam_msgs::WayPoint v_1;
		v_1.ps.pose.position.x = p.ps.pose.position.x - curr.ps.pose.position.x;
		v_1.ps.pose.position.y = p.ps.pose.position.y - curr.ps.pose.position.y;
		v_1.ps.pose.position.z = 0;
		v_1.angle = 0;
		double norm1 = utility.PointLength(v_1.ps.pose.position);
		adam_msgs::WayPoint v_2;
		v_2.ps.pose.position.x = next.ps.pose.position.x - curr.ps.pose.position.x;
		v_2.ps.pose.position.y = next.ps.pose.position.y - curr.ps.pose.position.y;
		v_2.ps.pose.position.z = 0;
		v_2.angle = 0;
		double norm2 = utility.PointLength(v_2.ps.pose.position);
		double dot_pro = v_1.ps.pose.position.x * v_2.ps.pose.position.x + v_1.ps.pose.position.y * v_2.ps.pose.position.y;
		double a = utility.rad2deg(acos(dot_pro / (norm1 * norm2))); //余弦定理求角度
		if (a <= 90)
			min_index = min_index + 1;
	}

	return min_index;
}

/**
 * @brief 
 * @return 0: 路径终点，1: 接近终点，2: 路径中
 */
int TrajectoryGen::ExtractPartFromPointToDistanceDirectionFast(const std::vector<adam_msgs::WayPoint> &originalPath,
																															 const adam_msgs::WayPoint &pos,
																															 std::vector<adam_msgs::WayPoint> &extractedPath)
{ //提取离点最近的部分路径
	if (originalPath.size() < 2)
		return 0;

	extractedPath.clear();

	int close_index = GetClosestNextPointIndexDirectionFast(originalPath, pos, 0);
	double d = 0;

	if (close_index >= originalPath.size() - 2)
	{
		//close_index = originalPath.size() - 2;
		return 1;
	}

	for (int i = close_index; i >= 0; i--)
	{																																	 //extractedPath加入到close点距离为10的路径
		extractedPath.insert(extractedPath.begin(), originalPath.at(i)); //反向插入最近点到起点
		if (i < originalPath.size())
			d += utility.getPlaneDistance(originalPath.at(i).ps.pose.position, originalPath.at(i + 1).ps.pose.position);
		if (d > 10)
			break;
	}

	//extractedPath.push_back(info.perp_point);
	d = 0;
	for (int i = close_index + 1; i < (int)originalPath.size(); i++)
	{ //extractedPath加入close点后minDistance的路径

		extractedPath.push_back(originalPath.at(i));
		if (i > 0)
			d += utility.getPlaneDistance(originalPath.at(i).ps.pose.position, originalPath.at(i - 1).ps.pose.position);
		if (d > m_PlanningParams.planDistance)
			break;
	}

	if (extractedPath.size() < 2)
	{
		std::cout << std::endl
							<< "### Planner Z . Extracted Rollout Path is too Small, Size = " << extractedPath.size() << std::endl;
		return 0;
	}
	FixPathDensity(extractedPath, m_PlanningParams.pathDensity);
	CalcAngleAndCost(extractedPath);
	std::vector<adam_msgs::WayPoint> end_interpolation;

	for (int i = 0; i < m_PlanningParams.lastInterpolationNumber; i++)
	{
		end_interpolation.insert(end_interpolation.begin(), extractedPath.at(extractedPath.size() - 1));
		extractedPath.pop_back();
	}

	double deceleration = utility.Deceleration(utility.getPlaneDistance(end_interpolation.at(0).ps.pose.position, end_interpolation.at(end_interpolation.size() - 1).ps.pose.position), extractedPath.at(0).vel_x);
	FixPathDensity(end_interpolation, 0.2);
	ROS_INFO_STREAM("inter " << end_interpolation.size() << " deceleration: " << deceleration);

	adam_msgs::WayPoint start = end_interpolation.at(0);
	adam_msgs::WayPoint end = end_interpolation.at(end_interpolation.size() - 1);
	for (int i = 0; i < end_interpolation.size(); i++)
	{

		end_interpolation.at(i).ps.pose.position.x = start.ps.pose.position.x + (end.ps.pose.position.x - start.ps.pose.position.x) / (end_interpolation.size() - 1) * i;
		end_interpolation.at(i).ps.pose.position.y = start.ps.pose.position.y + (end.ps.pose.position.y - start.ps.pose.position.y) / (end_interpolation.size() - 1) * i;
		end_interpolation.at(i).cost = start.cost + (end.cost - start.cost) / (end_interpolation.size() - 1) * i;
		end_interpolation.at(i).angle = start.angle + (end.angle - start.angle) / (end_interpolation.size() - 1) * i;
		if (i > 0)
		{
			end_interpolation.at(i).vel_x = std::max(0.0, sqrt(pow(end_interpolation.at(i - 1).vel_x, 2) - 2 * deceleration * utility.getPlaneDistance(end_interpolation.at(i - 1).ps.pose.position, end_interpolation.at(i).ps.pose.position)));
			if (end_interpolation.at(0).vel_x < 0)
				end_interpolation.at(i).vel_x = -end_interpolation.at(i).vel_x;
		}
		ROS_INFO_STREAM("i: " << i << " vel_x: " << end_interpolation.at(i).vel_x);

		extractedPath.push_back(end_interpolation.at(i));
	}
	return 2;
}

void TrajectoryGen::FixPathDensity(std::vector<adam_msgs::WayPoint> &path, const double &distanceDensity)
{ //修正path中每个点的间隔
	if (path.size() <= 1 || distanceDensity == 0)
		return;

	double d = 0, a = 0;
	double margin = distanceDensity * 0.01;
	double remaining = 0;
	int nPoints = 0;
	std::vector<adam_msgs::WayPoint> fixedPath;
	fixedPath.push_back(path.at(0));
	for (unsigned int si = 0, ei = 1; ei < path.size();)
	{
		d += utility.getPlaneDistance(path.at(ei).ps.pose.position, path.at(ei - 1).ps.pose.position) + remaining; //ei总距离
		a = utility.AngleBetweenTwoPoints(path.at(si).ps.pose.position, path.at(ei).ps.pose.position);						 //ei偏离si的弧度

		if (d < distanceDensity - margin) // skip
		{
			ei++;
			remaining = 0;
		}
		else if (d > (distanceDensity + margin)) // skip
		{
			adam_msgs::WayPoint pm = path.at(si);
			nPoints = d / distanceDensity;
			for (int k = 0; k < nPoints; k++)
			{
				pm.ps.pose.position.x = pm.ps.pose.position.x + distanceDensity * cos(a);
				pm.ps.pose.position.y = pm.ps.pose.position.y + distanceDensity * sin(a);
				fixedPath.push_back(pm);
			}
			remaining = d - nPoints * distanceDensity;
			si++;
			path.at(si).ps.pose.position = pm.ps.pose.position;
			d = 0;
			ei++;
		}
		else
		{
			d = 0;
			remaining = 0;
			fixedPath.push_back(path.at(ei));
			ei++;
			si = ei - 1;
		}
	}

	path = fixedPath;
}

void TrajectoryGen::GenerateRunoffTrajectory(const std::vector<adam_msgs::WayPoint> &referencePath,
																						 const adam_msgs::WayPoint &carPos,
																						 const double &speed,
																						 std::vector<std::vector<adam_msgs::WayPoint>> &rollOutPaths)
{

	if (referencePath.size() == 0)
		return;
	if (m_PlanningParams.rollOutDistance <= 0)
		return;
	rollOutPaths.clear();

	std::vector<std::vector<adam_msgs::WayPoint>> temp_rollOutPath;
	int startIndex = 0, endIndex = 0;
	std::vector<double> e_distances;

	TrajectoryGen::CalculateRollInTrajectories(carPos, speed, referencePath, startIndex, endIndex,
																						 e_distances, temp_rollOutPath);

	rollOutPaths = temp_rollOutPath;
}

void TrajectoryGen::CalculateRollInTrajectories(const adam_msgs::WayPoint &carPos,
																								const double &speed,
																								const std::vector<adam_msgs::WayPoint> &CenterPath,
																								int &start_index,
																								int &end_index,
																								std::vector<double> &end_laterals,
																								std::vector<std::vector<adam_msgs::WayPoint>> &rollInPaths)
{
	adam_msgs::WayPoint wp;

	//????
	int iLimitIndex = (m_PlanningParams.carTipMargin / 0.3) / m_PlanningParams.pathDensity;
	if (iLimitIndex >= CenterPath.size())
		iLimitIndex = CenterPath.size() - 1;

	//Get Closest Index
	RelativeInfo info;
	GetRelativeInfo(CenterPath, carPos, info, 0);
	double remaining_distance = 0;
	int close_index = info.iBack;
	//printf("iFront =%d ,iBack =%d\n",info.iFront,info.iBack);
	for (unsigned int i = close_index; i < CenterPath.size() - 1; i++)
	{
		if (i > 0)
			remaining_distance += utility.getPlaneDistance(CenterPath[i].ps.pose.position, CenterPath[i + 1].ps.pose.position);
	}

	double initial_roll_in_distance = info.perp_distance;

	//calculate the starting index
	double temp_distance = 0;
	unsigned int far_index = close_index;

	//calculate end index
	double start_distance = m_PlanningParams.rollInSpeedFactor * speed + m_PlanningParams.rollInMargin;
	if (start_distance > remaining_distance)
		start_distance = remaining_distance;

	temp_distance = 0;
	for (unsigned int i = close_index; i < CenterPath.size(); i++)
	{
		if (i > 0)
			temp_distance += utility.getPlaneDistance(CenterPath[i].ps.pose.position, CenterPath[i - 1].ps.pose.position);

		far_index = i; //far_index = min(i,CenterPath.size())
		if (temp_distance >= start_distance)
		{
			//far_index = i;
			break;
		}
	}

	int centralTrajectoryIndex = m_PlanningParams.rollOutNumber / 2;
	std::vector<double> end_distance_list;
	for (int i = 0; i <= m_PlanningParams.rollOutNumber; i++)
	{
		double end_roll_in_distance = m_PlanningParams.rollOutDensity * (i - centralTrajectoryIndex);
		end_distance_list.push_back(end_roll_in_distance);
	}

	start_index = close_index;
	end_index = far_index;
	end_laterals = end_distance_list;

	//calculate the actual calculation starting index
	temp_distance = 0;
	unsigned int smoothing_start_index = start_index;
	unsigned int smoothing_end_index = end_index;

	for (unsigned int i = smoothing_start_index; i < CenterPath.size(); i++)
	{
		if (i > 0)
			temp_distance += utility.getPlaneDistance(CenterPath[i].ps.pose.position, CenterPath[i - 1].ps.pose.position);
		if (temp_distance > m_PlanningParams.carTipMargin)
			break;

		smoothing_start_index++;
	}

	temp_distance = 0;
	for (unsigned int i = smoothing_end_index; i < CenterPath.size(); i++)
	{
		if (i > 0)
			temp_distance += utility.getPlaneDistance(CenterPath[i].ps.pose.position, CenterPath[i - 1].ps.pose.position);
		if (temp_distance > m_PlanningParams.carTipMargin)
			break;

		smoothing_end_index++;
	}

	int nSteps = end_index - smoothing_start_index; //rollin规划距离

	std::vector<double> inc_list;
	rollInPaths.clear();
	std::vector<double> inc_list_inc;
	for (int i = 0; i <= m_PlanningParams.rollOutNumber; i++)
	{
		double diff = end_laterals.at(i) - initial_roll_in_distance; //当前位置到各个路径的垂直距离
		inc_list.push_back(diff / (double)nSteps);
		rollInPaths.push_back(std::vector<adam_msgs::WayPoint>());
		inc_list_inc.push_back(0);
	}

	std::vector<std::vector<adam_msgs::WayPoint>> execluded_from_smoothing;
	for (unsigned int i = 0; i <= m_PlanningParams.rollOutNumber; i++)
		execluded_from_smoothing.push_back(std::vector<adam_msgs::WayPoint>());

	//Insert First strait points within the tip of the car range
	for (unsigned int j = start_index; j < smoothing_start_index; j++)
	{
		wp = CenterPath.at(j);
		double original_speed = wp.vel_x;
		for (unsigned int i = 0; i <= m_PlanningParams.rollOutNumber; i++)
		{
			wp.ps.pose.position.x = CenterPath.at(j).ps.pose.position.x - initial_roll_in_distance * cos(wp.angle + M_PI_2);
			wp.ps.pose.position.y = CenterPath.at(j).ps.pose.position.y - initial_roll_in_distance * sin(wp.angle + M_PI_2);
			if (i != centralTrajectoryIndex)
				wp.vel_x = original_speed * m_PlanningParams.laneChangeSpeedFactor;
			else
				wp.vel_x = original_speed;

			if (j < iLimitIndex)
				execluded_from_smoothing.at(i).push_back(wp);
			else
				rollInPaths.at(i).push_back(wp);
		}
	}

	for (unsigned int j = smoothing_start_index; j < end_index; j++)
	{
		wp = CenterPath.at(j);
		double original_speed = wp.vel_x;
		for (unsigned int i = 0; i <= m_PlanningParams.rollOutNumber; i++)
		{
			inc_list_inc[i] += inc_list[i];
			double d = inc_list_inc[i];
			wp.ps.pose.position.x = CenterPath.at(j).ps.pose.position.x - initial_roll_in_distance * cos(wp.angle + M_PI_2) - d * cos(wp.angle + M_PI_2);
			wp.ps.pose.position.y = CenterPath.at(j).ps.pose.position.y - initial_roll_in_distance * sin(wp.angle + M_PI_2) - d * sin(wp.angle + M_PI_2);
			if (i != centralTrajectoryIndex)
				wp.vel_x = original_speed * m_PlanningParams.laneChangeSpeedFactor;
			else
				wp.vel_x = original_speed;

			rollInPaths.at(i).push_back(wp);
		}
	}

	//Insert last strait points to make better smoothing
	for (unsigned int j = end_index; j < smoothing_end_index; j++)
	{
		wp = CenterPath.at(j);
		double original_speed = wp.vel_x;
		for (unsigned int i = 0; i <= m_PlanningParams.rollOutNumber; i++)
		{
			double d = end_laterals.at(i);
			wp.ps.pose.position.x = CenterPath.at(j).ps.pose.position.x - d * cos(wp.angle + M_PI_2);
			wp.ps.pose.position.y = CenterPath.at(j).ps.pose.position.y - d * sin(wp.angle + M_PI_2);
			if (i != centralTrajectoryIndex)
				wp.vel_x = original_speed * m_PlanningParams.laneChangeSpeedFactor;
			else
				wp.vel_x = original_speed;
			rollInPaths.at(i).push_back(wp);
		}
	}

	for (unsigned int i = 0; i <= m_PlanningParams.rollOutNumber; i++)
		rollInPaths.at(i).insert(rollInPaths.at(i).begin(), execluded_from_smoothing.at(i).begin(), execluded_from_smoothing.at(i).end());

	temp_distance = 0;
	for (unsigned int j = smoothing_end_index; j < CenterPath.size(); j++) //rollout
	{
		if (j > 0)
			temp_distance += utility.getPlaneDistance(CenterPath.at(j).ps.pose.position, CenterPath.at(j - 1).ps.pose.position);

		if (temp_distance > m_PlanningParams.rollOutDistance) //rollout距离
			break;

		wp = CenterPath.at(j);
		double original_speed = wp.vel_x;
		for (unsigned int i = 0; i < rollInPaths.size(); i++)
		{
			double d = end_laterals.at(i);
			wp.ps.pose.position.x = CenterPath.at(j).ps.pose.position.x - d * cos(wp.angle + M_PI_2);
			wp.ps.pose.position.y = CenterPath.at(j).ps.pose.position.y - d * sin(wp.angle + M_PI_2);

			if (i != centralTrajectoryIndex)
				wp.vel_x = original_speed * m_PlanningParams.laneChangeSpeedFactor;
			else
				wp.vel_x = original_speed;

			rollInPaths.at(i).push_back(wp);
		}
	}

	for (unsigned int i = 0; i <= m_PlanningParams.rollOutNumber; i++)
	{ //平滑生成的路径
		SmoothPath(rollInPaths.at(i), m_PlanningParams.smoothingDataWeight, m_PlanningParams.smoothingSmoothWeight, m_PlanningParams.smoothingToleranceError);
	}
}

void TrajectoryGen::SmoothPath(std::vector<adam_msgs::WayPoint> &path,
															 double weight_data,
															 double weight_smooth,
															 double tolerance)
{

	if (path.size() <= 2)
	{
		//cout << "Can't Smooth Path, Path_in Size=" << path.size() << endl;
		return;
	}

	const std::vector<adam_msgs::WayPoint> &path_in = path;
	std::vector<adam_msgs::WayPoint> smoothPath_out = path_in;

	double change = tolerance;
	double xtemp, ytemp;
	int nIterations = 0;

	int size = path_in.size();

	while (change >= tolerance)
	{
		change = 0.0;
		for (int i = 1; i < size - 1; i++)
		{
			//			if (smoothPath_out[i].pos.a != smoothPath_out[i - 1].pos.a)
			//				continue;

			xtemp = smoothPath_out[i].ps.pose.position.x;
			ytemp = smoothPath_out[i].ps.pose.position.y;

			smoothPath_out[i].ps.pose.position.x += weight_data * (path_in[i].ps.pose.position.x - smoothPath_out[i].ps.pose.position.x);
			smoothPath_out[i].ps.pose.position.y += weight_data * (path_in[i].ps.pose.position.y - smoothPath_out[i].ps.pose.position.y);

			smoothPath_out[i].ps.pose.position.x += weight_smooth * (smoothPath_out[i - 1].ps.pose.position.x + smoothPath_out[i + 1].ps.pose.position.x - (2.0 * smoothPath_out[i].ps.pose.position.x));
			smoothPath_out[i].ps.pose.position.y += weight_smooth * (smoothPath_out[i - 1].ps.pose.position.y + smoothPath_out[i + 1].ps.pose.position.y - (2.0 * smoothPath_out[i].ps.pose.position.y));

			change += fabs(xtemp - smoothPath_out[i].ps.pose.position.x);
			change += fabs(ytemp - smoothPath_out[i].ps.pose.position.y);
		}
		nIterations++;
	}

	path = smoothPath_out;
}

bool TrajectoryGen::GetRelativeInfo(const std::vector<adam_msgs::WayPoint> &trajectory,
																		const adam_msgs::WayPoint &wp,
																		RelativeInfo &info,
																		const int &prevIndex)
{
	if (trajectory.size() < 2)
		return false;

	adam_msgs::WayPoint wp0; //最近的前一点
	adam_msgs::WayPoint wp1; //后一点
	if (trajectory.size() == 2)
	{
		wp0 = trajectory.at(0);
		wp1.ps.pose.position.x = (trajectory.at(0).ps.pose.position.x + trajectory.at(1).ps.pose.position.x) / 2.0;
		wp1.ps.pose.position.y = (trajectory.at(0).ps.pose.position.y + trajectory.at(1).ps.pose.position.y) / 2.0;
		wp1.ps.pose.position.z = (trajectory.at(0).ps.pose.position.z + trajectory.at(1).ps.pose.position.z) / 2.0;
		info.iFront = 1;
		info.iBack = 0;
	}
	else
	{
		info.iFront = GetClosestNextPointIndexDirectionFast(trajectory, wp, prevIndex);

		if (info.iFront > 0)
			info.iBack = info.iFront - 1;
		else
			info.iBack = 0;

		if (info.iFront == 0)
		{
			wp0 = trajectory.at(info.iFront);
			wp1 = trajectory.at(info.iFront + 1);
		}
		else if (info.iFront > 0 && info.iFront < trajectory.size() - 1)
		{
			wp0 = trajectory.at(info.iFront - 1);
			wp1 = trajectory.at(info.iFront);
		}
		else
		{
			wp0 = trajectory.at(info.iFront - 1);
			wp1.ps.pose.position.x = (trajectory.at(info.iFront - 1).ps.pose.position.x + trajectory.at(info.iFront).ps.pose.position.x) / 2.0;
			wp1.ps.pose.position.y = (trajectory.at(info.iFront - 1).ps.pose.position.y + trajectory.at(info.iFront).ps.pose.position.y) / 2.0;
			wp1.ps.pose.position.z = (trajectory.at(info.iFront - 1).ps.pose.position.y + trajectory.at(info.iFront).ps.pose.position.z) / 2.0;
		}
	}

	adam_msgs::WayPoint prevWP = wp0;
	Mat3 rotationMat(-wp1.angle);
	Mat3 translationMat(-wp.ps.pose.position.x, -wp.ps.pose.position.y);
	Mat3 invRotationMat(wp1.angle);
	Mat3 invTranslationMat(wp.ps.pose.position.x, wp.ps.pose.position.y);

	wp0 = translationMat * wp0;
	wp0 = rotationMat * wp0;

	wp1 = translationMat * wp1;
	wp1 = rotationMat * wp1;

	double m = (wp1.ps.pose.position.y - wp0.ps.pose.position.y) / (wp1.ps.pose.position.x - wp0.ps.pose.position.x);
	info.perp_distance = wp1.ps.pose.position.y - m * wp1.ps.pose.position.x; // solve for x = 0

	if (std::isnan(info.perp_distance) || std::isinf(info.perp_distance)) //(isnan 不是数字返回1,是数字返回0),(isinf 无限大返回1,非无限大返回0)
		info.perp_distance = 0;

	info.to_front_distance = fabs(wp1.ps.pose.position.x); // distance on the x axes //垂点到wp1的距离

	info.perp_point = wp1;
	info.perp_point.ps.pose.position.x = 0;									 // on the same y axis of the car
	info.perp_point.ps.pose.position.y = info.perp_distance; //perp distance between the car and the trajectory

	info.perp_point = invRotationMat * info.perp_point;
	info.perp_point = invTranslationMat * info.perp_point; //垂点

	info.from_back_distance = utility.getPlaneDistance(info.perp_point.ps.pose.position, prevWP.ps.pose.position); //垂点到wp2的距离
	//info.angle_diff = utility.rad2deg(utility.AngleBetweenTwoAnglesPositive(wp1.angle, wp.angle));

	return true;
}

TrajectoryCost TrajectoryGen::DoOneStep(const std::vector<std::vector<adam_msgs::WayPoint>> &rollOuts,
																				const std::vector<adam_msgs::WayPoint> &totalPaths,
																				const adam_msgs::WayPoint &currState,
																				const int &currIndex,
																				const double &m_Steer)
{
	TrajectoryCost bestTrajectory;
	bestTrajectory.bBlocked = false;
	bestTrajectory.closest_obj_distance = m_PlanningParams.planDistance;
	bestTrajectory.closest_obj_velocity = 0;
	bestTrajectory.index = 0;

	if (m_PrevCostIndex == -1)
		m_PrevCostIndex = m_PlanningParams.rollOutNumber / 2;

	m_TrajectoryCosts.clear();

	//将所有路径的所有rollout都push到m_TrajectoryCosts中
	if (rollOuts.size() > 0)
	{
		std::vector<TrajectoryCost> costs = CalculatePriorityCosts(rollOuts);
		m_TrajectoryCosts = costs;
	}

	CalculateTransitionCosts(m_TrajectoryCosts, currIndex);

	adam_msgs::WayPoint p;

	CalculateObstacleCosts(m_TrajectoryCosts, rollOuts, currState);

	NormalizeCosts(m_TrajectoryCosts);

	int smallestIndex = -1;
	double smallestCost = 1;
	double smallestDistance = DBL_MAX;

	//cout << "Trajectory Costs Log : CurrIndex: " << currIndex << " --------------------- " << endl;
	for (unsigned int i = 0; i < m_TrajectoryCosts.size(); i++)
	{
		//cout << m_TrajectoryCosts.at(ic).ToString();
		if (m_TrajectoryCosts.at(i).cost < smallestCost)
		{
			smallestCost = m_TrajectoryCosts.at(i).cost;
			smallestIndex = i;
		}
		if (m_TrajectoryCosts.at(i).closest_obj_distance < smallestDistance)
			smallestDistance = m_TrajectoryCosts.at(i).closest_obj_distance;
	}

	//All is blocked !
	if (smallestIndex == -1 && m_PrevCostIndex < (int)m_TrajectoryCosts.size())
	{
		bestTrajectory.bBlocked = true;
		bestTrajectory.index = currIndex;
		bestTrajectory.closest_obj_distance = smallestDistance;
		//bestTrajectory.index = smallestIndex;
		if (smallestDistance < 3)
			for (int p = 0; p < m_RollOuts.at(currIndex).size(); p++)
			{
				m_RollOuts.at(currIndex).at(p).vel_x = 0;
			}
	}
	else if (smallestIndex >= 0)
	{
		bestTrajectory = m_TrajectoryCosts.at(smallestIndex);
	}

	m_PrevCostIndex = smallestIndex;

	return bestTrajectory;
}

void TrajectoryGen::NormalizeCosts(std::vector<TrajectoryCost> &trajectoryCosts)
{
	//Normalize costs
	double totalPriorities = 0;
	double totalTransitions = 0;
	double totalObjects = 0;

	for (unsigned int ic = 0; ic < trajectoryCosts.size(); ic++)
	{
		totalPriorities += trajectoryCosts.at(ic).priority_cost;
		totalTransitions += trajectoryCosts.at(ic).transition_cost;
		totalObjects += trajectoryCosts.at(ic).closest_obj_cost;
	}

	//	cout << "------ Normalizing Step " << endl;
	for (unsigned int ic = 0; ic < trajectoryCosts.size(); ic++)
	{
		if (totalPriorities != 0)
			trajectoryCosts.at(ic).priority_cost = trajectoryCosts.at(ic).priority_cost / totalPriorities;
		else
			trajectoryCosts.at(ic).priority_cost = 0;

		if (totalTransitions != 0)
			trajectoryCosts.at(ic).transition_cost = trajectoryCosts.at(ic).transition_cost / totalTransitions;
		else
			trajectoryCosts.at(ic).transition_cost = 0;

		if (totalObjects != 0)
			trajectoryCosts.at(ic).closest_obj_cost = trajectoryCosts.at(ic).closest_obj_cost / totalObjects;
		else
			trajectoryCosts.at(ic).closest_obj_cost = 0;

		trajectoryCosts.at(ic).priority_cost = m_PlanningParams.weightPriority * trajectoryCosts.at(ic).priority_cost;
		trajectoryCosts.at(ic).transition_cost = m_PlanningParams.weightTransition * trajectoryCosts.at(ic).transition_cost;
		trajectoryCosts.at(ic).closest_obj_cost = m_PlanningParams.weightObj * trajectoryCosts.at(ic).closest_obj_cost;

		trajectoryCosts.at(ic).cost = trajectoryCosts.at(ic).priority_cost +
																	trajectoryCosts.at(ic).transition_cost +
																	trajectoryCosts.at(ic).closest_obj_cost +
																	trajectoryCosts.at(ic).obj;

		// std::cout.setf(std::ios::left);
		// std::cout << "Index: " 		<<std::setfill(' ')<<std::setw(2)	<< ic
		// 		<< ", Priority: " 	<<std::setfill(' ')<<std::setw(10)	<<trajectoryCosts.at(ic).priority_cost
		// 		<< ", Transition: " <<std::setfill(' ')<<std::setw(10) 	<<trajectoryCosts.at(ic).transition_cost
		// 		<< ", obj: "		<<std::setfill(' ')<<std::setw(10) 	<<trajectoryCosts.at(ic).closest_obj_cost
		// 		<< ", Avg: " 		<<std::setfill(' ')<<std::setw(10) 	<<trajectoryCosts.at(ic).cost
		// 		<< ", bool "		<<trajectoryCosts.at(ic).obj
		// 		<< std::endl;
	}

	//	cout << "------------------------ " << endl;
}

std::vector<TrajectoryCost> TrajectoryGen::CalculatePriorityCosts(const std::vector<std::vector<adam_msgs::WayPoint>> &laneRollOuts)
{ //计算各支路cost
	std::vector<TrajectoryCost> costs;
	TrajectoryCost tc;
	int centralIndex = m_PlanningParams.rollOutNumber / 2;

	for (unsigned int i = 0; i < laneRollOuts.size(); i++)
	{
		tc.index = i;
		tc.relative_index = i - centralIndex;
		tc.distance_from_center = m_PlanningParams.rollOutDensity * tc.relative_index;
		tc.priority_cost = fabs(tc.distance_from_center);				 //cost即为距离中心路径距离
		tc.closest_obj_distance = m_PlanningParams.planDistance; //障碍物距离即为预瞄距离

		costs.push_back(tc);
	}

	return costs;
}

bool TrajectoryGen::detectCollision(const adam_msgs::WayPoint &wp)
{
	// Define the robot as rectangle
	double left = -1.0 * m_CarBasicInfo.width / 2.0;
	double right = m_CarBasicInfo.width / 2.0;
	double top = m_CarBasicInfo.length / 2.0;
	double bottom = -1.0 * m_CarBasicInfo.length / 2.0;
	double resolution = map_info_.resolution;

	// Coordinate of base_link in OccupancyGrid frame
	int index_x, index_y;
	poseToIndex(wp.ps.pose, &index_x, &index_y);
	double base_x = index_x * resolution;
	double base_y = index_y * resolution;
	// Convert each point to index and check if the node is Obstacle
	for (double x = left; x < right; x += resolution)
	{
		for (double y = top; y > bottom; y -= resolution)
		{

			// 2D point rotation
			int index_x = (x + base_x) / resolution;
			int index_y = (y + base_y) / resolution;
			if (isOutOfRange(index_x, index_y))
			{
				return true;
			}
			if (is_Obstacle_nodes[index_y][index_x] == true)
			{
				return true;
			}
		}
	}

	return false;
}

void TrajectoryGen::poseToIndex(const geometry_msgs::Pose &pose, int *index_x, int *index_y)
{
	*index_x = (pose.position.x - map_info_.origin.position.x) / map_info_.resolution;
	*index_y = (pose.position.y - map_info_.origin.position.y) / map_info_.resolution;
}

void TrajectoryGen::CalculateTransitionCosts(std::vector<TrajectoryCost> &trajectoryCosts, const int &currTrajectoryIndex)
{ //计算变道所需的cost
	for (int i = 0; i < trajectoryCosts.size(); i++)
	{
		trajectoryCosts.at(i).transition_cost = fabs(m_PlanningParams.rollOutDensity * (i - currTrajectoryIndex));
	}
}

void TrajectoryGen::CalculateObstacleCosts(std::vector<TrajectoryCost> &trajectoryCosts,
																					 const std::vector<std::vector<adam_msgs::WayPoint>> &rollOuts,
																					 const adam_msgs::WayPoint &m_CurrentPos)
{ //计算变道所需的cost
	for (int i = 0; i < m_PlanningParams.rollOutNumber + 1; i++)
	{
		trajectoryCosts.at(i).obj = false;
		for (int j = 0; j < rollOuts.at(i).size(); j++)
		{
			if (detectCollision(rollOuts.at(i).at(j)))
			{

				for (int k = 0; k <= m_PlanningParams.rollOutNumber; k++)
				{
					int dis = abs(i - k);
					if (dis <= m_PlanningParams.rollOutNumber / 3)
						trajectoryCosts.at(k).closest_obj_cost += m_PlanningParams.rollOutNumber + 1 - dis;
				}
				trajectoryCosts.at(i).closest_obj_distance = utility.getPlaneDistance(rollOuts.at(i).at(j).ps.pose.position, m_CurrentPos.ps.pose.position);

				trajectoryCosts.at(i).obj = true;
				if (i > 0)
					trajectoryCosts.at(i - 1).obj = true;
				if (i < m_PlanningParams.rollOutNumber)
					trajectoryCosts.at(i + 1).obj = true;
				break;
			}
		}
	}
}

bool TrajectoryGen::isOutOfRange(int index_x, int index_y)
{
	if (index_x < 0 || index_x >= static_cast<int>(map_info_.width) || index_y < 0 ||
			index_y >= static_cast<int>(map_info_.height))
		return true;

	return false;
}

void TrajectoryGen::callbackGetCarBasicInfo(const adam_msgs::CarBasicInfoConstPtr &msg)
{
	m_CarBasicInfo = *msg;
	bCarBasicInfo = true;
}

void TrajectoryGen::callbackGetCarLimitInfo(const adam_msgs::CarLimitInfoConstPtr &msg)
{
	m_CarLimitInfo = *msg;
	bCarLimitInfo = true;
}

void TrajectoryGen::callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
	if (!bInitPos)
	{
		m_InitPos = msg->pose.pose;
		m_CurrentPos.ps.pose = m_InitPos;
		m_CurrentPos.angle = tf::getYaw(msg->pose.pose.orientation);
		bInitPos = true;
	}
}

void TrajectoryGen::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
	m_CurrentPos.ps.pose = msg->pose;
	m_InitPos = m_CurrentPos.ps.pose;
	m_CurrentPos.angle = tf::getYaw(msg->pose.orientation);
	bCurrentPos = true;
	bInitPos = true;
}

// void TrajectoryGen::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr &msg)
// {
// 	m_Speed = msg->twist.twist.linear.x;
// 	m_Steer += atan(m_CarBasicInfo.wheel_base * msg->twist.twist.angular.z / msg->twist.twist.linear.x);
// 	bVehicleStatus = true;
// }

void TrajectoryGen::callbackGetGlobalPlannerPath(const nav_msgs::PathConstPtr &msg)
{
	m_temp_path.clear();
	adam_msgs::WayPoint wp;
	for (unsigned int i = 0; i < msg->poses.size(); i++)
	{
		wp.ps.pose = msg->poses[i].pose;
		m_temp_path.push_back(wp);
	}
	SmoothPath(m_temp_path, m_PlanningParams.smoothingDataWeight, m_PlanningParams.smoothingSmoothWeight, m_PlanningParams.smoothingToleranceError);
	CalcAngleAndCost(m_temp_path);

	bool bOldGlobalPath = m_GlobalPath.size() == m_temp_path.size();

	if (bOldGlobalPath)
	{
		bOldGlobalPath = CompareTrajectories(m_GlobalPath, m_temp_path);
	}
	if (!bOldGlobalPath)
	{
		bGlobalPath = true;
		m_GlobalPath = m_temp_path;
		std::cout << "Received New Global Path Generator ! " << std::endl;
	}
}

void TrajectoryGen::callbackGetObstaclesMap(const nav_msgs::OccupancyGridConstPtr &msg)
{
	bCostMap = true;
	map_info_ = msg->info;
	is_Obstacle_nodes.clear();
	is_Obstacle_nodes.resize(msg->info.height);

	for (int i = 0; i < msg->info.height; i++)
		is_Obstacle_nodes[i].resize(msg->info.width);
	int count = 0;

	for (size_t i = 0; i < msg->info.height; i++)
	{
		for (size_t j = 0; j < msg->info.width; j++)
		{
			// Index of subscribing OccupancyGrid message
			size_t og_index = i * msg->info.width + j;
			int cost = msg->data[og_index];

			// hc is set to be 0 when reset()

			if (cost > m_PlanningParams.obstacleThreshold)
			{
				is_Obstacle_nodes[i][j] = true;
				count++;
			}
			else
				is_Obstacle_nodes[i][j] = false;
		}
	}
}

void TrajectoryGen::MainLoop()
{
	ROS_INFO_STREAM("local_plannner start");
	ros::Rate loop_rate(20);

	while (ros::ok())
	{
		ros::spinOnce();

		if (!bInitPos)
		{
			ROS_WARN_STREAM("InitPos topics are not subscribed yet ... ");
			loop_rate.sleep();
			continue;
		}
		else if (!bCostMap)
		{
			ROS_WARN_STREAM("CostMap topics are not subscribed yet ... ");
			loop_rate.sleep();
			continue;
		}
		else if (!bGlobalPath)
		{
			ROS_WARN_STREAM("GlobalPath topics are not subscribed yet ... ");
			loop_rate.sleep();
			continue;
		}
		else if (!bCarBasicInfo || !bCarLimitInfo)
		{
			ROS_WARN_STREAM("Car info topics are not subscribed yet ... ");
			loop_rate.sleep();
			continue;
		}

		TrajectoryCost tc;
		nav_msgs::Path pub_path;
		geometry_msgs::Twist pub_twist;
		m_GlobalPathSection.clear();

		int end_flag = ExtractPartFromPointToDistanceDirectionFast(m_GlobalPath, m_CurrentPos, m_GlobalPathSection);
		if (end_flag == 0)
		{
			ROS_INFO_STREAM("Local planning had end,please publish new global path ... ");
			pub_twist.linear.x = 0;
			pub_twist.angular.z = 0;
			pub_LocalTrajectoriesSpeed.publish(pub_twist);
			bGlobalPath = false;
			loop_rate.sleep();
			continue;
		}
		GenerateRunoffTrajectory(m_GlobalPathSection, m_CurrentPos, m_Speed, m_RollOuts);
		if (m_GlobalPathSection.size() > 0)
		{
			tc = DoOneStep(m_RollOuts, m_GlobalPathSection, m_CurrentPos, m_PrevCostIndex, m_Steer);
			pub_path.poses.clear();
			pub_path.header.stamp = ros::Time::now();
			pub_path.header.frame_id = "/map";
			ROS_INFO("choose no.%d path", tc.index);
			if (tc.index < 0 && bLocalPath == false)
			{
				for (int i = 0; i < m_GlobalPath.size(); i++)
				{
					geometry_msgs::PoseStamped ps;
					ps.pose = m_GlobalPath.at(i).ps.pose;
					pub_path.poses.push_back(ps);
				}
				tc.index = m_PlanningParams.rollOutNumber / 2;
				pub_twist.linear.x = m_RollOuts.at(tc.index).at(1).vel_x;

				bLocalPath = true;
			}
			else
			{
				for (int i = 0; i < m_RollOuts.at(tc.index).size(); i++)
				{
					geometry_msgs::PoseStamped ps;
					ps.pose = m_RollOuts.at(tc.index).at(i).ps.pose;
					pub_path.poses.push_back(ps);
					//ROS_INFO("vel = %f ,angle1 = %f", m_RollOuts.at(tc.index).at(i).vel_x,(m_RollOuts.at(tc.index).at(i).angle)*RAD2DEG);
				}
				pub_twist.linear.x = m_RollOuts.at(tc.index).at(1).vel_x;
				ROS_INFO("vel = %f ,angle1 = %f", m_RollOuts.at(tc.index).at(1).vel_x, utility.rad2deg(fabs(m_RollOuts.at(tc.index).at(1).angle - m_RollOuts.at(tc.index).at(0).angle)));
			}
		}

		if (end_flag == 1)
		{
			if (m_RollOuts.at(tc.index).size() < 5)
			{
				pub_twist.linear.x *= 0.5;
			} else if (m_RollOuts.at(tc.index).size() < 10){
				pub_twist.linear.x *= 0.75;
			}
		}

		visualization_msgs::MarkerArray all_rollOuts;
		visualization_msgs::MarkerArray all_speeds;
		visualization_msgs::Marker current_speed;
		visualization_msgs::Marker car_model;

		TrajectoriesToMarkers(m_RollOuts, all_rollOuts, tc);
		TrajectoriesSpeedToMarkers(m_RollOuts, all_speeds, tc);
		CurrentSpeedToMarkers(pub_twist.linear.x, current_speed);
		CarModelToMarker(car_model);

		pub_LocalTrajectories.publish(pub_path);
		pub_LocalTrajectoriesSpeed.publish(pub_twist);
		pub_LocalTrajectoriesRviz.publish(all_rollOuts);
		pub_LocalTrajectoriesSpeedRviz.publish(all_speeds);
		pub_LocalTrajectoriesCurrentSpeedRviz.publish(current_speed);
		pub_CarModelRviz.publish(car_model);

		loop_rate.sleep();
	}
}

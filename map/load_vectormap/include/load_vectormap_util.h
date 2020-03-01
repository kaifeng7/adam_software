/*
 * @Author: fengk 
 * @Date: 2019-04-03 12:19:44 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-10 15:32:31
 */
#include<geometry_msgs/PoseStamped.h>
#include<adam_msgs/WayPointStamped.h>
#include<tf/tf.h>

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
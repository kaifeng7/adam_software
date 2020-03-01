#ifndef __USER_PROTOCOL__
#define __USER_PROTOCOL__

#include <cstdio>

#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

struct pose
{
  double x, y, z;
  double pitch, roll, yaw;
  pose() {}
  pose(double x, double y, double z, double roll, double pitch, double yaw) : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw)
  {
  }
  void reset()
  {
    x = 0.;
    y = 0.;
    z = 0.;
    roll = 0.;
    pitch = 0.;
    yaw = 0.;
  }
  pose operator+(const pose &b) const
  {
    return pose(x + b.x, y + b.y, z + b.z, roll + b.roll, pitch + b.pitch, yaw + b.yaw);
  }
  pose operator-(const pose &b) const
  {
    return pose(x - b.x, y - b.y, z - b.z, roll - b.roll, pitch - b.pitch, yaw - b.yaw);
  }
  pose operator+=(const pose &b)
  {
    return *this + b;
  }
  pose operator-=(const pose &b)
  {
    return *this - b;
  }
};

bool geometryPose2Pose(const geometry_msgs::Pose &from, pose &to)
{
  to.x = from.position.x;
  to.y = from.position.y;
  to.z = from.position.z;
  tf::Matrix3x3(tf::Quaternion(from.orientation.x, from.orientation.y, from.orientation.z, from.orientation.w))
      .getEulerYPR(to.yaw, to.pitch, to.roll);

  return true;
}

bool pose2GeometryPose(geometry_msgs::Pose &to, const pose &from)
{
  to.position.x = from.x;
  to.position.y = from.y;
  to.position.z = from.z;
  to.orientation = tf::createQuaternionMsgFromRollPitchYaw(from.roll, from.pitch, from.yaw);

  return true;
}

/*protocol*/
enum _ptl_type
{
	VAL_START = 0,
	VAL_POSE,
	VAL_IMU_RAW,
	VAL_VEL,
	VAL_PID,
	VAL_IMU,
	VAL_ARM_POSE,
	VAL_ARM_AXIS,
	VAL_END
};
#define _PACKET_SYN_CODE_START 0xFA
static const uint16_t PACKET_SIZE = 96;
#pragma pack(1)
typedef struct _packet_data
{
	uint8_t syn;
	uint8_t type;
	union {
		struct
		{
			float liner[3], angular[4];
		} vel;
		struct
		{
			bool rot_ok, acc_ok, mag_ok;
			double rot[3], acc[3], mag[3];
		} imu;
		float pid[3];
	} dat;
	uint8_t syn_CR;
	uint8_t syn_LF;
} PacketData;
#pragma pack()

#endif
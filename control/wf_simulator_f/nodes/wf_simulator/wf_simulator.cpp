/*
 * @Author: fengk 
 * @Date: 2019-04-12 15:09:07 
 * @Last Modified by: fengk
 * @Last Modified time: 2019-04-12 15:19:15
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <iostream>
#include <std_msgs/Int32.h>
#include <random>
#include <adam_msgs/VehicleCmd.h>

#include "waypoint_follower/libwaypoint_follower.h"

namespace
{
geometry_msgs::Twist _current_velocity;

const std::string SIMULATION_FRAME = "sim_base_link";
const std::string MAP_FRAME = "map";

geometry_msgs::Pose _initial_pose;
bool _initial_set = false;
bool _pose_set = false;

ros::Publisher g_odometry_publisher;
ros::Publisher g_velocity_publisher;

double g_position_error;
double g_angle_error;

constexpr int LOOP_RATE = 50; // 50Hz

void controlCmdCallBack(const adam_msgs::VehicleCmdConstPtr &msg)
{
  _current_velocity.linear.x = msg->ctrl_cmd.speed;
  _current_velocity.angular.z = msg->ctrl_cmd.steering_angle;
}

void getTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform)
{
  static tf::TransformListener listener;

  while (1)
  {
    try
    {
      listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
      break;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }
}

void initialposeCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &input)
{
  tf::StampedTransform transform;
  getTransformFromTF(MAP_FRAME, input->header.frame_id, transform);

  _initial_pose.position.x = input->pose.pose.position.x + transform.getOrigin().x();
  _initial_pose.position.y = input->pose.pose.position.y + transform.getOrigin().y();
  _initial_pose.position.z = input->pose.pose.position.z + transform.getOrigin().z();
  _initial_pose.orientation = input->pose.pose.orientation;

  _initial_set = true;
  _pose_set = false;
}


void publishOdometry()
{
  static ros::Time current_time = ros::Time::now();
  static ros::Time last_time = ros::Time::now();
  static geometry_msgs::Pose pose;
  static double th = 0;
  static tf::TransformBroadcaster odom_broadcaster;

  if (!_pose_set)
  {
    pose.position = _initial_pose.position;
    pose.orientation = _initial_pose.orientation;
    th = tf::getYaw(pose.orientation);
    ROS_INFO_STREAM("pose set : (" << pose.position.x << " " << pose.position.y << " " << pose.position.z << " " << th
                                   << ")");
    _pose_set = true;
  }

  double vx = _current_velocity.linear.x;
  double vth = _current_velocity.angular.z;
  current_time = ros::Time::now();

  // compute odometry in a typical way given the velocities of the robot
  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::uniform_real_distribution<double> rnd_dist(0.0, 2.0);
  double rnd_value_x = rnd_dist(mt) - 1.0;
  double rnd_value_y = rnd_dist(mt) - 1.0;
  double rnd_value_th = rnd_dist(mt) - 1.0;

  double dt = (current_time - last_time).toSec();
  double delta_x = (vx * cos(th)) * dt + rnd_value_x * g_position_error;
  double delta_y = (vx * sin(th)) * dt + rnd_value_y * g_position_error;
  double delta_th = vth * dt + rnd_value_th * g_angle_error * M_PI / 180;

  pose.position.x += delta_x;
  pose.position.y += delta_y;
  th += delta_th;
  pose.orientation = tf::createQuaternionMsgFromYaw(th);

  // std::cout << "delta (x y th) : (" << delta_x << " " << delta_y << " " << delta_th << ")" << std::endl;
  // std::cout << "current_velocity(linear.x angular.z) : (" << _current_velocity.linear.x << " " <<
  // _current_velocity.angular.z << ")"<< std::endl;
  //    std::cout << "current_pose : (" << pose.position.x << " " << pose.position.y<< " " << pose.position.z << " " <<
  //    th << ")" << std::endl << std::endl;

  // first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = MAP_FRAME;
  odom_trans.child_frame_id = SIMULATION_FRAME;

  odom_trans.transform.translation.x = pose.position.x;
  odom_trans.transform.translation.y = pose.position.y;
  odom_trans.transform.translation.z = pose.position.z;
  odom_trans.transform.rotation = pose.orientation;

  // send the transform
  odom_broadcaster.sendTransform(odom_trans);

  // next, we'll publish the odometry message over ROS
  std_msgs::Header h;
  h.stamp = current_time;
  h.frame_id = MAP_FRAME;

  geometry_msgs::PoseStamped ps;
  ps.header = h;
  ps.pose = pose;

  geometry_msgs::TwistStamped ts;
  ts.header = h;
  ts.twist.linear.x = vx;
  ts.twist.angular.z = vth;

  // publish the message
  g_odometry_publisher.publish(ps);
  g_velocity_publisher.publish(ts);

  last_time = current_time;
}
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "wf_simulator");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param("position_error", g_position_error, double(0.0));
  pnh.param("angle_error", g_angle_error, double(0.0));

  // publish topic
  g_odometry_publisher = nh.advertise<geometry_msgs::PoseStamped>("sim_pose", 10);
  g_velocity_publisher = nh.advertise<geometry_msgs::TwistStamped>("sim_velocity", 10);

  // subscribe topic
  ros::Subscriber control_cmd_subscriber = nh.subscribe("/cmd_vel", 10, controlCmdCallBack);
  ros::Subscriber initialpose_subscriber;

  
  initialpose_subscriber = nh.subscribe("initialpose", 10, initialposeCallback);

  ros::Rate loop_rate(LOOP_RATE);
  while (ros::ok())
  {
    ros::spinOnce();  // check subscribe topic

    if (!_initial_set)
    {
      loop_rate.sleep();
      continue;
    }

    publishOdometry();

    loop_rate.sleep();
  }

  return 0;
}

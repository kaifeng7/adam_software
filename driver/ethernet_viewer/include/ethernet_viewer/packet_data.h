/**
 * @file packet_data.h
 * @author heng zhang (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2018-11-26
 * 
 * @copyright Copyright (c) 2018
 * 
 */
#ifndef __VIEWER_PACKET_DATA__
#define __VIEWER_PACKET_DATA__

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <cstring>

namespace packet_data
{
using namespace std;
namespace enc = sensor_msgs::image_encodings;

// frame_id 最大长度 31
const uint8_t L_FRAME_ID = 32;

// 定义不同的数据类型
const uint8_t T_POINT_CLOUD = 0;
const uint8_t T_CLOUD_MAP = 1;
const uint8_t T_CAMERA_IMAGE = 2;
const uint8_t T_VEHICLE_STATE = 3;
const uint8_t T_GLOBAL_PATH = 4;
const uint8_t T_LOCAL_PATH = 5;
const uint8_t T_HDMAP = 6;

struct Header
{
    long stamp;
    char frame_id[L_FRAME_ID];
    int seq;

    Header(){};
    Header(long stamp, string f, int seq) : stamp(stamp), seq(seq)
    {
        strcpy(frame_id, f.c_str());
    }
    Header &operator=(const std_msgs::Header &h)
    {
        stamp = h.stamp.toSec() * 1000;
        strcpy(frame_id, h.frame_id.c_str());
        seq = h.seq;
        return *this;
    }
};

struct Point
{
    float x, y, z, i;

    Point(){};
    Point(float x, float y, float z) : x(x), y(y), z(z), i(0.)
    {
    }
    Point(float x, float y, float z, float i) : x(x), y(y), z(z), i(i)
    {
    }
};

struct PointCloud
{
    uint8_t type;
    Header header;
    int size;
    // Point points[500000];
    Point *points;

    PointCloud(){};
    PointCloud(const sensor_msgs::PointCloud2ConstPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI> pcl_pc;
        pcl::fromROSMsg(*msg, pcl_pc);
        type = T_POINT_CLOUD;
        header = msg->header;
        size = pcl_pc.points.size();

        points = new Point[size];
        for (int i = 0; i < size; ++i)
        {
            points[i] = Point(pcl_pc.points[i].x, pcl_pc.points[i].y, pcl_pc.points[i].z, pcl_pc.points[i].intensity);
        }
    }
};

struct CameraImage
{
    uint8_t type;
    Header header;
    uint32_t height;
    uint32_t width;
    uint32_t byte_depth;
    uint32_t num_channels;
    uint8_t is_bigendian;
    uint32_t step;
    uint8_t *data;

    CameraImage(){};
    CameraImage(const sensor_msgs::ImageConstPtr msg)
    {
        type = T_CAMERA_IMAGE;
        header = msg->header;
        height = msg->height;
        width = msg->width;
        byte_depth = enc::bitDepth(msg->encoding) / 8;
        num_channels = enc::numChannels(msg->encoding);
        is_bigendian = msg->is_bigendian;
        step = msg->step;

        assert(msg->data.size() == height * step);
        data = new uint8_t[height * step];
        for (int i = 0; i < msg->data.size(); ++i)
        {
            data[i] = msg->data[i];
        }
    }
};

} // namespace packet_data

#endif
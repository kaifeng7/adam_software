#include <boost/foreach.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/io/bag_io.h>

using namespace std;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

string param_bag_filename_; // bag 文件位置
string param_dest_dir_;     // 解析数据的存放位置
string param_img_prefix_;   // 解析图片的前缀名
string param_pc_prefix_;    // 解析点云的前缀名
string param_img_topic_;
string param_pc_topic_;

rosbag::Bag bag_;
rosbag::View view_;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bag2raw_data");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<string>("bag_filename", param_bag_filename_, "./");
  pnh.param<string>("dest_dir", param_dest_dir_, "");
  pnh.param<string>("img_prefix", param_img_prefix_, "img_");
  pnh.param<string>("pc_prefix", param_pc_prefix_, "pc_");
  pnh.param<string>("img_topic", param_img_topic_, "/cv_cam/image_raw/compressed");
  pnh.param<string>("pc_topic", param_pc_topic_, "/lslidar_point_cloud");

  if (param_dest_dir_.empty())
  {
    ROS_ERROR("Please set dest_dir to save data!");
    return 0;
  }
  try
  {
    ROS_INFO("Openning bag file:%s", param_bag_filename_.c_str());
    ROS_INFO("Saving data to: %s", param_dest_dir_.c_str());
    bag_.open(param_bag_filename_, rosbag::bagmode::Read);
  }
  catch (const ros::Exception &e)
  {
    ROS_ERROR("Error in openning bag_file: %s", e.what());
    exit(-1);
  }

  vector<string> topics;
  topics.push_back(param_img_topic_);
  topics.push_back(param_pc_topic_);
  view_.addQuery(bag_, rosbag::TopicQuery(topics));

  int cnt_img = 0, cnt_pc = 0;
  string filename;
  stringstream ss;
  BOOST_FOREACH (rosbag::MessageInstance const m, view_)
  {
    ss.str("");
    if (m.getTopic() == param_img_topic_ || "/" + m.getTopic() == param_img_topic_)
    {
      sensor_msgs::CompressedImageConstPtr i_img = m.instantiate<sensor_msgs::CompressedImage>();
      // 借鉴于 image_tranport_plugin/compressed_subscriber.cpp
      cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
      // Copy message header
      cv_ptr->header = i_img->header;
      namespace enc = sensor_msgs::image_encodings;
      try
      {
        cv_ptr->image = cv::imdecode(cv::Mat(i_img->data), cv::IMREAD_COLOR);
        const size_t split_pos = i_img->format.find(';');
        if (split_pos == std::string::npos)
        {
          // Older version of compressed_image_transport does not signal image format
          switch (cv_ptr->image.channels())
          {
          case 1:
            cv_ptr->encoding = enc::MONO8;
            break;
          case 3:
            cv_ptr->encoding = enc::BGR8;
            break;
          default:
            ROS_ERROR("Unsupported number of channels: %i", cv_ptr->image.channels());
            break;
          }
        }
        else
        {
          std::string image_encoding = i_img->format.substr(0, split_pos);

          cv_ptr->encoding = image_encoding;

          if (enc::isColor(image_encoding))
          {
            std::string compressed_encoding = i_img->format.substr(split_pos);
            bool compressed_bgr_image = (compressed_encoding.find("compressed bgr") != std::string::npos);

            // Revert color transformation
            if (compressed_bgr_image)
            {
              // if necessary convert colors from bgr to rgb
              if ((image_encoding == enc::RGB8) || (image_encoding == enc::RGB16))
                cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGB);

              if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16))
                cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGBA);

              if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16))
                cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2BGRA);
            }
            else
            {
              // if necessary convert colors from rgb to bgr
              if ((image_encoding == enc::BGR8) || (image_encoding == enc::BGR16))
                cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGR);

              if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16))
                cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGRA);

              if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16))
                cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2RGBA);
            }
          }
        }
      }
      catch (const ros::Exception &e)
      {
        ROS_ERROR("Error in cv copy: %s", e.what());
      }
      size_t rows = cv_ptr->image.rows;
      size_t cols = cv_ptr->image.cols;
      if (rows > 0 && cols > 0)
      {
        // 文件名格式：dir + img_prefix + 000000 + "_" + date(unix date 乘以 100) + ".jpg"
        ss << param_dest_dir_ << "/" << param_img_prefix_ << setw(6) << setfill('0') << cnt_img << "_" << long(i_img->header.stamp.toSec() * 100) << ".jpg";
        cv::imwrite(ss.str(), cv_ptr->image);
        ++cnt_img;
        if (cnt_img % 100 == 0)
        {
          ROS_INFO("Saved %d images.", cnt_img);
        }
      }
    }

    if (m.getTopic() == param_pc_topic_ || "/" + m.getTopic() == param_pc_topic_)
    {
      sensor_msgs::PointCloud2ConstPtr i_pc = m.instantiate<sensor_msgs::PointCloud2>();
      PointCloudT o_pc = PointCloudT();
      pcl::fromROSMsg(*i_pc, o_pc);
      ss << param_dest_dir_ << "/" << param_pc_prefix_ << setw(6) << setfill('0') << cnt_pc << "_" << long(i_pc->header.stamp.toSec() * 100) << ".pcd";
      pcl::io::savePCDFile(ss.str(), o_pc);
      ++cnt_pc;
      if (cnt_pc % 100 == 0)
      {
        ROS_INFO("Saved %d pc.", cnt_pc);
      }
    }
  }

  bag_.close();

  return 0;
}

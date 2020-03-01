#ifndef PARAM_GETTER_H_
#define PARAM_GETTER_H_

#include <unistd.h>
#include <string>
#include <ros/ros.h>

namespace drive_control {

typedef struct
{ 
  bool debug_mode;
  int hz;
  int num_timesteps;
  int num_iters;
  float x_pos;
  float y_pos;
  float heading;
  float gamma;
  float init_steering;
  float init_throttle;
  float steering_std;
  float throttle_std;
  float max_throttle;
  std::string model_path;
} SystemParams;

inline bool fileExists (const std::string& name) {
    return ( access( name.c_str(), F_OK ) != -1 );
}

template <typename T>
T getRosParam(std::string paramName, ros::NodeHandle nh)
{
  std::string key;
  T val;
  bool found = nh.searchParam(paramName, key);
  if (!found){
    ROS_ERROR("Could not find parameter name '%s' in tree of node '%s'", 
              paramName.c_str(), nh.getNamespace().c_str());
  }
  else {
    nh.getParam(key, val);
  }
  return val;
}

void loadParams(SystemParams* params, ros::NodeHandle nh);

}

#endif /*PARAM_GETTER_H_*/


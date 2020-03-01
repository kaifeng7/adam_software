#include "drive_control_x/path_integral/param_getter.h"

namespace drive_control
{
void loadParams(SystemParams *params, ros::NodeHandle nh)
{
    params->debug_mode = getRosParam<bool>("debug_mode", nh);
    params->hz = getRosParam<int>("hz", nh);
    params->num_timesteps = getRosParam<int>("num_timesteps", nh);
    params->num_iters = getRosParam<int>("num_iters", nh);
    params->x_pos = getRosParam<double>("x_pos", nh);
    params->y_pos = getRosParam<double>("y_pos", nh);
    params->heading = getRosParam<double>("heading", nh);
    params->gamma = getRosParam<double>("gamma", nh);
    params->init_steering = getRosParam<double>("init_steering", nh);
    params->init_throttle = getRosParam<double>("init_throttle", nh);
    params->steering_std = getRosParam<double>("steering_std", nh);
    params->throttle_std = getRosParam<double>("throttle_std", nh);
    params->max_throttle = getRosParam<double>("max_throttle", nh);
    params->model_path = getRosParam<std::string>("model_path", nh);
}
} // namespace drive_control
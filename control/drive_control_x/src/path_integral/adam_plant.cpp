#include <cstdio>
#include <cstdlib>

#include "drive_control_x/path_integral/adam_plant.h"

namespace drive_control
{
AdamPlant::AdamPlant(ros::NodeHandle global_node, ros::NodeHandle mppi_node,
                     bool debug_mode, int hz, bool nodelet)
{
    nodeNamespace_ = mppi_node.getNamespace();
    std::string pose_estimate_name = getRosParam<std::string>("pose_estimate", mppi_node);
    std::string trajectory_topic_name_ = getRosParam<std::string>("trajectory_topic", mppi_node);
    debug_mode_ = getRosParam<bool>("debug_mode", mppi_node);
    numTimesteps_ = getRosParam<int>("num_timesteps", mppi_node);
    useFeedbackGains_ = getRosParam<bool>("use_feedback_gains", mppi_node);
    throttleMax_ = getRosParam<float>("max_throttle", mppi_node);
    deltaT_ = 1.0 / hz;

    controlSequence_.resize(CONTROL_DIM * numTimesteps_);
    stateSequence_.resize(STATE_DIM * numTimesteps_);

    //Initialize the publishers.
    control_pub_ = mppi_node.advertise<adam_msgs::chassisCommand>("chassisCommand", 1);
    path_pub_ = mppi_node.advertise<nav_msgs::Path>("nominalPath", 1);
    subscribed_pose_pub_ = mppi_node.advertise<nav_msgs::Odometry>("subscribedPose", 1);
    status_pub_ = mppi_node.advertise<adam_msgs::pathIntegralStatus>("mppiStatus", 1);
    timing_data_pub_ = mppi_node.advertise<adam_msgs::pathIntegralTiming>("timingInfo", 1);

    rollout_path_pub_ = mppi_node.advertise<visualization_msgs::MarkerArray>("rolloutPath", 1);

    //Initialize the subscribers.
    pose_sub_ = global_node.subscribe(pose_estimate_name, 1, &AdamPlant::poseCall, this);
    servo_sub_ = global_node.subscribe("chassisState", 1, &AdamPlant::servoCall, this);
    trajectory_sub_ = global_node.subscribe(trajectory_topic_name_, 1, &AdamPlant::globalTrajectoryCall, this);

    //Timer callback for path publisher
    pathTimer_ = mppi_node.createTimer(ros::Duration(0.033), &AdamPlant::pubPath, this);
    statusTimer_ = mppi_node.createTimer(ros::Duration(0.033), &AdamPlant::pubStatus, this);
    debugImgTimer_ = mppi_node.createTimer(ros::Duration(0.033), &AdamPlant::displayDebugImage, this);
    timingInfoTimer_ = mppi_node.createTimer(ros::Duration(0.033), &AdamPlant::pubTimingData, this);

    //Initialize auxiliary variables.
    safe_speed_zero_ = false;
    debug_mode_ = debug_mode;
    activated_ = false;
    new_model_available_ = false;
    last_pose_call_ = ros::Time::now();

    //Initialize yaw derivative to zero
    full_state_.yaw_mder = 0.0;
    status_ = 1;
    if (debug_mode_)
    {
        ocs_msg_ = "Debug Mode";
    }
    else
    {
        ocs_msg_ = "";
    }
    std::string info = "MPPI Controller";
    std::string hardwareID = "";
    std::string portPath = "";

    //Debug image display signaller
    receivedDebugImg_ = false;
    is_nodelet_ = nodelet;

    if (!debug_mode_)
    {
        ROS_INFO("DEBUG MODE is set to FALSE, waiting to receive first pose estimate...  ");
    }
    else
    {
        ROS_WARN("DEBUG MODE is set to TRUE. DEBUG MODE must be FALSE in order to be launched from a remote machine. \n");
    }
}

void AdamPlant::setSolution(std::vector<float> traj, std::vector<float> controls, util::EigenAlignedVector<float, 2, 7> gains, ros::Time ts, double loop_speed)
{
    boost::mutex::scoped_lock lock(access_guard_);
    optimizationLoopTime_ = loop_speed;
    solutionTs_ = ts;
    for (int t = 0; t < numTimesteps_; t++)
    {
        for (int i = 0; i < STATE_DIM; i++)
        {
            stateSequence_[t * STATE_DIM + i] = traj[t * STATE_DIM + i];
        }
        for (int i = 0; i < CONTROL_DIM; i++)
        {
            controlSequence_[t * CONTROL_DIM + i] = traj[t * CONTROL_DIM + i];
        }
    }
    feedback_gains_ = gains;
    solutionReceived_ = true;
}

void AdamPlant::setTimingInfo(double poseDiff, double tickTime, double sleepTime)
{
    boost::mutex::scoped_lock lock(access_guard_);
    timingData_.averageTimeBetweenPoses = poseDiff;
    timingData_.averageOptimizationCycleTime = tickTime;
    timingData_.averageSleepTime = sleepTime;
}

void AdamPlant::pubTimingData(const ros::TimerEvent &)
{
    boost::mutex::scoped_lock lock(access_guard_);
    timingData_.header.stamp = ros::Time::now();
    timing_data_pub_.publish(timingData_);
}

void AdamPlant::setDebugImage(cv::Mat img)
{
    receivedDebugImg_ = true;
    boost::mutex::scoped_lock lock(access_guard_);

    debugImg_ = img;
}

// //添加path显示
// void AdamPlant::setDebugImage(cv::Mat img, float x_pos, float y_pos)
// {
//     receivedDebugImg_ = true;
//     boost::mutex::scoped_lock lock(access_guard_);
//     //图像分辨率500x500
//     int ppm = 50;
//     int img_h = 500;
//     int img_w = 500;
//     for(int i = 0; i < numTimesteps_; i++){
//         int p_x = (stateSequence_[i * AUTORALLY_STATE_DIM] - x_pos) * ppm + img_w / 2;
//         int p_y = (stateSequence_[i * AUTORALLY_STATE_DIM + 1] - y_pos) * ppm + img_h / 2;
//         cv::Point pathPoint(p_x, p_y);
//         cv::circle(img, pathPoint, 2, cv::Scalar(0,0,255));
//     }
//     debugImg_ = img;
// }

void AdamPlant::rvizShow(float* points, int size){

    visualization_msgs::MarkerArray pathArray;
    for(int i = 0; i < size; i++){
        visualization_msgs::Marker points_v;
        std_msgs::ColorRGBA color;
        color.a = 1.0;
        color.b = 0.0;
        color.g = 0.0;
        color.r = 1.0;

        points_v.header.stamp = ros::Time::now();
        points_v.header.frame_id = "odom";
        points_v.ns ="~";
        points_v.id = 0;
        points_v.type = visualization_msgs::Marker::POINTS;
        points_v.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Pose pose;
        pose.orientation.w = 1;
        points_v.pose = pose;
        points_v.color = color;
        points_v.scale.x = 0.1;
        points_v.scale.y = 0.1;
        points_v.scale.z = 0.1;
        points_v.frame_locked = true;
        
    }

    // for (int i = 0; i < NUM_ROLLOUTS; i++)
    // {
    //     visualization_msgs::Marker rollout_path_v;
    //     std_msgs::ColorRGBA color;
    //     color.a = 1.0;
    //     color.b = 0.0;
    //     color.r = 1.0;
    //     color.g = 0.0;
    //     rollout_path_v.header.stamp = begin;
    //     rollout_path_v.header.frame_id = "odom";
    //     rollout_path_v.ns = "~";
    //     rollout_path_v.id = 0;
    //     rollout_path_v.type = visualization_msgs::Marker::POINTS;
    //     rollout_path_v.action = visualization_msgs::Marker::ADD;
    //     geometry_msgs::Pose pose;
    //     pose.orientation.w = 1;
    //     rollout_path_v.pose = pose;
    //     rollout_path_v.color = color;
    //     rollout_path_v.scale.x = 0.1;
    //     rollout_path_v.scale.y = 0.1;
    //     rollout_path_v.scale.z = 0.1;
    //     rollout_path_v.frame_locked = true;

    //     for (int j = 0; j < 100; j++)
    //     {
    //         geometry_msgs::Point pathPoint;
    //         pathPoint.x = rollout_state_[i * 600 + j * 6];
    //         pathPoint.y = rollout_state_[i * 600 + j * 6 + 1];
    //         rollout_path_v.points.push_back(pathPoint);

    //     }
    //     pathArray.markers.push_back(rollout_path_v);
    // }
    // std::cout << "markers: " << pathArray.markers.size() << std::endl;
    // rollout_path_pub_.publish(pathArray);
}

void AdamPlant::displayDebugImage(const ros::TimerEvent &)
{
    if (receivedDebugImg_.load() && !is_nodelet_)
    {
        boost::mutex::scoped_lock lock(access_guard_);
        cv::namedWindow(nodeNamespace_, cv::WINDOW_AUTOSIZE);
        cv::imshow(nodeNamespace_, debugImg_);
    }
    if (receivedDebugImg_.load() && !is_nodelet_)
    {
        cv::waitKey(1);
    }
}

void AdamPlant::poseCall(nav_msgs::Odometry pose_msg)
{
    if (poseCount_ == 0)
    {
        ROS_INFO("First pose estimate received. \n");
    }
    // std::cout << "pose" << pose_msg.pose.pose.position.x << pose_msg.pose.pose.position.y << std::endl;
    boost::mutex::scoped_lock lock(access_guard_);
    // Update the timestamp
    last_pose_call_ = pose_msg.header.stamp;
    poseCount_++;
    // Set activated to true --> we are receiving state messages.
    activated_ = true;
    // Update positon
    full_state_.x_pos = pose_msg.pose.pose.position.x;
    full_state_.y_pos = pose_msg.pose.pose.position.y;
    full_state_.z_pos = pose_msg.pose.pose.position.z;
    // Grab the quaternion
    float q0 = pose_msg.pose.pose.orientation.w;
    float q1 = pose_msg.pose.pose.orientation.x;
    float q2 = pose_msg.pose.pose.orientation.y;
    float q3 = pose_msg.pose.pose.orientation.z;
    // Update euler angles.
    full_state_.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, q3 * q3 - q2 * q2 - q1 * q1 + q0 * q0);
    full_state_.pitch = -asin(2 * q1 * q3 - 2 * q0 * q2);
    full_state_.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, q1 * q1 + q0 * q0 - q3 * q3 - q2 * q2);
    // Don't allow heading to wrap around
    if (last_heading_ > 3.0 && full_state_.yaw < -3.0)
    {
        heading_multiplier_ += 1;
    }
    else if (last_heading_ < -3.0 && full_state_.yaw > 3.0)
    {
        heading_multiplier_ -= 1;
    }
    last_heading_ = full_state_.yaw;
    full_state_.yaw = full_state_.yaw + heading_multiplier_ * 2 * PI;

    // Update hte quaternion
    full_state_.q0 = q0;
    full_state_.q1 = q1;
    full_state_.q2 = q2;
    full_state_.q3 = q3;
    // Update the world frame velocity
    full_state_.x_vel = pose_msg.twist.twist.linear.x;
    full_state_.y_vel = pose_msg.twist.twist.linear.y;
    full_state_.z_vel = pose_msg.twist.twist.linear.z;
    // Update the body frame longtitudenal and lateral velocity
    full_state_.u_x = cos(full_state_.yaw) * full_state_.x_vel + sin(full_state_.yaw) * full_state_.y_vel;
    full_state_.u_y = -sin(full_state_.yaw) * full_state_.x_vel + cos(full_state_.yaw) * full_state_.y_vel;
    // Update the minus yaw derivative.
    full_state_.yaw_mder = -pose_msg.twist.twist.angular.z;

    // Interpolate and publish the current control
    double timeFromLastOpt = (last_pose_call_ - solutionTs_).toSec();

    if (solutionReceived_ && timeFromLastOpt > 0 && timeFromLastOpt < (numTimesteps_ - 1) * deltaT_)
    {
        double steering_ff, throttle_ff, steering_fb, throttle_fb, steering, throttle;
        int lowerIdx = (int)(timeFromLastOpt / deltaT_);
        int upperIdx = lowerIdx + 1;
        double alpha = (timeFromLastOpt - lowerIdx * deltaT_) / deltaT_;                                     //小数部分
        steering_ff = (1 - alpha) * controlSequence_[2 * lowerIdx] + alpha * controlSequence_[2 * upperIdx]; //距离lowerIdx越远，下个时间点的权重越大
        throttle_ff = (1 - alpha) * controlSequence_[2 * lowerIdx + 1] + alpha * controlSequence_[2 * upperIdx + 1];

        if (!useFeedbackGains_)
        {
            //Just publish the computed open loop controls
            steering = steering_ff;
            throttle = throttle_ff;
        }
        else
        {
            //Compute the error between the current and actual state and apply feedback gains
            Eigen::MatrixXf current_state(7, 1);
            Eigen::MatrixXf desired_state(7, 1);
            Eigen::MatrixXf deltaU;
            current_state << full_state_.x_pos, full_state_.y_pos, full_state_.yaw, full_state_.roll, full_state_.u_x, full_state_.u_y, full_state_.yaw_mder;
            for (int i = 0; i < 7; i++)
            {
                desired_state(i) = (1 - alpha) * stateSequence_[7 * lowerIdx + i] + alpha * stateSequence_[7 * upperIdx + i];
            }

            //更相信最接近的feddback_gains_
            deltaU = ((1 - alpha) * feedback_gains_[lowerIdx] + alpha * feedback_gains_[upperIdx]) * (current_state - desired_state);

            if (std::isnan(deltaU(0)) || std::isnan(deltaU(1)))
            {
                steering = steering_ff;
                throttle = throttle_ff;
            }
            else
            {
                steering_fb = deltaU(0);
                throttle_fb = deltaU(1);
                steering = fmin(0.99, fmax(-0.99, steering_ff + steering_fb));
                throttle = fmin(throttleMax_, fmax(-0.99, throttle_ff + throttle_fb));
            }
        }
        pubControl(steering, throttle);
    }
}

void AdamPlant::servoCall(adam_msgs::chassisState servo_msg)
{
    boost::mutex::scoped_lock lock(access_guard_);
    full_state_.steering = servo_msg.steering;
    full_state_.throttle = servo_msg.throttle;
}

void AdamPlant::runstopCall(adam_msgs::runstop safe_msg)
{
    boost::mutex::scoped_lock lock(access_guard_);
    if (safe_msg.motionEnabled == false)
    {
        safe_speed_zero_ = true;
    }
}

void AdamPlant::globalTrajectoryCall(nav_msgs::Path traj_msgs){

    traj_msg_ = traj_msgs;

    traj_received_ = true;
            printf("adamplant.cpp:%d, %f, %f, %f\n", 1600, (float)traj_msg_.poses[1600].pose.position.x, (float)traj_msg_.poses[1600].pose.position.y, (float)traj_msg_.poses[1600].pose.position.z);

}
 
nav_msgs::Path AdamPlant::getTrajPath(){

    return traj_msg_;
}

void AdamPlant::pubPath(const ros::TimerEvent &)
{
    boost::mutex::scoped_lock lock(access_guard_);
    path_msg_.poses.clear();
    nav_msgs::Odometry subscribed_state;
    int i;
    float phi, theta, psi, q0, q1, q2, q3;
    ros::Time begin = solutionTs_;
    for (int i = 0; i < numTimesteps_; i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = stateSequence_[i * (STATE_DIM)];
        pose.pose.position.y = stateSequence_[i * (STATE_DIM) + 1];
        pose.pose.position.z = 0;
        psi = stateSequence_[i * (STATE_DIM) + 2]; //yaw
        phi = stateSequence_[i * (STATE_DIM) + 3]; //roll
        theta = 0;
        q0 = cos(phi / 2) * cos(theta / 2) * cos(psi / 2) + sin(phi / 2) * sin(theta / 2) * sin(psi / 2);
        q1 = -cos(phi / 2) * sin(theta / 2) * sin(psi / 2) + cos(theta / 2) * cos(psi / 2) * sin(phi / 2);
        q2 = cos(phi / 2) * cos(psi / 2) * sin(theta / 2) + sin(phi / 2) * cos(theta / 2) * sin(psi / 2);
        q3 = cos(phi / 2) * cos(theta / 2) * sin(psi / 2) - sin(phi / 2) * cos(psi / 2) * sin(theta / 2);
        pose.pose.orientation.w = q0;
        pose.pose.orientation.x = q1;
        pose.pose.orientation.y = q2;
        pose.pose.orientation.z = q3;
        pose.header.stamp = begin + ros::Duration(i * deltaT_);
        pose.header.frame_id = "odom";
        path_msg_.poses.push_back(pose);
        if (i == 0)
        {
            subscribed_state.pose.pose = pose.pose;
            subscribed_state.twist.twist.linear.x = stateSequence_[4];
            subscribed_state.twist.twist.linear.y = stateSequence_[5];
            subscribed_state.twist.twist.angular.z = -stateSequence_[6];
        }
    }
    subscribed_state.header.stamp = begin;
    subscribed_state.header.frame_id = "odom";
    path_msg_.header.stamp = begin;
    path_msg_.header.frame_id = "odom";
    path_pub_.publish(path_msg_);
    subscribed_pose_pub_.publish(subscribed_state);

    //  可视化采样路径
    // visualization_msgs::MarkerArray pathArray;

    // for (int i = 0; i < NUM_ROLLOUTS; i++)
    // {
    //     visualization_msgs::Marker rollout_path_v;
    //     std_msgs::ColorRGBA color;
    //     color.a = 1.0;
    //     color.b = 0.0;
    //     color.r = 1.0;
    //     color.g = 0.0;
    //     rollout_path_v.header.stamp = begin;
    //     rollout_path_v.header.frame_id = "odom";
    //     rollout_path_v.ns = "~";
    //     rollout_path_v.id = 0;
    //     rollout_path_v.type = visualization_msgs::Marker::POINTS;
    //     rollout_path_v.action = visualization_msgs::Marker::ADD;
    //     geometry_msgs::Pose pose;
    //     pose.orientation.w = 1;
    //     rollout_path_v.pose = pose;
    //     rollout_path_v.color = color;
    //     rollout_path_v.scale.x = 0.1;
    //     rollout_path_v.scale.y = 0.1;
    //     rollout_path_v.scale.z = 0.1;
    //     rollout_path_v.frame_locked = true;

    //     for (int j = 0; j < 100; j++)
    //     {
    //         geometry_msgs::Point pathPoint;
    //         pathPoint.x = rollout_state_[i * 600 + j * 6];
    //         pathPoint.y = rollout_state_[i * 600 + j * 6 + 1];
    //         rollout_path_v.points.push_back(pathPoint);

    //     }
    //     pathArray.markers.push_back(rollout_path_v);
    // }
    // std::cout << "markers: " << pathArray.markers.size() << std::endl;
    // rollout_path_pub_.publish(pathArray);
}

void AdamPlant::setRolloutPath(std::vector<float> rolloutPath, int num_rollouts)
{
    NUM_ROLLOUTS = num_rollouts;
    rollout_state_ = rolloutPath;
    // std::cout << rolloutPath[0] << "," << rolloutPath[1] << std::endl;
//   std::cout << rolloutPath[102398] << "," << rolloutPath[102399] << std::endl;
}

void AdamPlant::pubControl(float steering, float throttle)
{
    adam_msgs::chassisCommand control_msg; ///< Control message initialization.
    //Publish the steering and throttle commands
    if (std::isnan(throttle) || std::isnan(steering))
    {
        //Nan control publish zeros and exit.
        ROS_INFO("NaN Control Input Detected");
        control_msg.steering = 0;
        control_msg.throttle = -.99;
        control_msg.frontBrake = -5.0;
        control_msg.header.stamp = ros::Time::now();
        control_msg.sender = "mppi_controller";
        control_pub_.publish(control_msg);
        ros::shutdown(); //No use trying to recover, quitting is the best option.
    }
    else
    {
        //Publish the computed control input.
        control_msg.steering = steering;
        control_msg.throttle = throttle;
        control_msg.frontBrake = -5.0;
        control_msg.header.stamp = ros::Time::now();
        control_msg.sender = "mppi_controller";
        control_pub_.publish(control_msg);
    }
}

void AdamPlant::pubStatus(const ros::TimerEvent &)
{
    boost::mutex::scoped_lock lock(access_guard_);
    status_msg_.info = ocs_msg_;
    status_msg_.status = status_;
    status_msg_.header.stamp = ros::Time::now();
    status_pub_.publish(status_msg_);
}

AdamPlant::FullState AdamPlant::getState()
{
    boost::mutex::scoped_lock lock(access_guard_);
    return full_state_;
}

bool AdamPlant::getRunstop()
{
    boost::mutex::scoped_lock lock(access_guard_);
    return safe_speed_zero_;
}

ros::Time AdamPlant::getLastPoseTime()
{
    boost::mutex::scoped_lock lock(access_guard_);
    return last_pose_call_;
}

int AdamPlant::checkStatus()
{
    boost::mutex::scoped_lock lock(access_guard_);
    if (!activated_)
    {
        status_ = 1;
        ocs_msg_ = "No pose estimates received.";
    }
    else if (safe_speed_zero_)
    {
        status_ = 1;
        ocs_msg_ = "Safe speed zero.";
    }
    else
    {
        ocs_msg_ = "Controller OK";
        status_ = 0; //Everything is good.
    }
    return status_;
}

void AdamPlant::dynRcfgCall(drive_control::PathIntegralParamsConfig &config, int lvl)
{
    boost::mutex::scoped_lock lock(access_guard_);
    costParams_.desired_speed = config.desired_speed;
    costParams_.speed_coefficient = config.speed_coefficient;
    costParams_.track_coefficient = config.track_coefficient;
    costParams_.max_slip_angle = config.max_slip_angle;
    costParams_.slip_penalty = config.slip_penalty;
    costParams_.crash_coefficient = config.crash_coefficient;
    costParams_.track_slop = config.track_slop;
    costParams_.steering_coeff = config.steering_coeff;
    costParams_.throttle_coeff = config.throttle_coeff;
    hasNewCostParams_ = true;
}

bool AdamPlant::hasNewDynRcfg()
{
    boost::mutex::scoped_lock lock(access_guard_);
    return hasNewCostParams_;
}

drive_control::PathIntegralParamsConfig AdamPlant::getDynRcfgParams()
{
    boost::mutex::scoped_lock lock(access_guard_);
    hasNewCostParams_ = false;
    return costParams_;
}

void AdamPlant::shutdown()
{
    //Shutdown timers, subscribers, and dynamic reconfigure
    boost::mutex::scoped_lock lock(access_guard_);
    path_pub_.shutdown();
    pose_sub_.shutdown();
    servo_sub_.shutdown();
    pathTimer_.stop();
    statusTimer_.stop();
    debugImgTimer_.stop();
    timingInfoTimer_.stop();
}

} // namespace drive_control

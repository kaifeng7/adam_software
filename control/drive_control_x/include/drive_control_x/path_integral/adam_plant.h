#ifndef ADAM_PLANT_H
#define ADAM_PLANT_H
#include <ros/ros.h>


// #include "drive_control_x/path_integral/mppi_controller.cuh"
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <atomic>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <eigen3/Eigen/Dense>

#include "adam_msgs/chassisCommand.h"
#include "adam_msgs/chassisState.h"
#include "adam_msgs/runstop.h"
#include "adam_msgs/pathIntegralTiming.h"
#include "adam_msgs/pathIntegralStatus.h"

#include "drive_control_x/ddp/util.h"
#include "drive_control_x/path_integral/param_getter.h"
#include "drive_control_x/PathIntegralParamsConfig.h"

namespace drive_control
{
#define PI 3.14159265359

/**
     * @class AdamPlant adam_plant.h
     * @brief Publishers and subscribers for the control system
     *
     * This class is treated as the plant for the MPPI controller. When the MPPI
     * controller has a control it sends to a functions in this class to receive 
     * state feedback. This class also publishes trajectory and spray information
     * and status information for both the controller and the OCS.
     */

class AdamPlant
{
  public:
    static const int STATE_DIM = 7;
    static const int CONTROL_DIM = 2;
    // Struct for holding the pose
    typedef struct
    {
        // x,y,z position
        float x_pos;
        float y_pos;
        float z_pos;
        // Eular angles
        float roll;
        float pitch;
        float yaw;
        // Quaternions
        float q0;
        float q1;
        float q2;
        float q3;
        // x, y, z velocity
        float x_vel;
        float y_vel;
        float z_vel;
        // Body frame velocity
        float u_x;
        float u_y;
        // Eular angle derivatives
        float yaw_mder;
        // Current control commads
        float steering;
        float throttle;
    } FullState;

    float last_heading_ = 0.0;
    float throttleMax_ = 0.99;
    int heading_multiplier_ = 0;

    boost::mutex access_guard_;
    std::string nodeNamespace_;

    bool new_model_available_;
    cv::Mat debugImg_;

     bool traj_received_ = false;   ///接收到跟随的路径信息

    bool solutionReceived_ = false;
    bool is_nodelet_;
    std::vector<float> controlSequence_;
    std::vector<float> stateSequence_;
    util::EigenAlignedVector<float, 2, 7> feedback_gains_; //矩阵vector，元素是2x7矩阵
    ros::Time solutionTs_;

    int numTimesteps_;
    double deltaT_;

    double optimizationLoopTime_;

    /**
     * @brief Constructor for AdamPlant, takes a ros node handle and initalizes publishers
     * and subscribers.
     * @param mppi_node A ros node handle.
     */
    AdamPlant(ros::NodeHandle global_node, ros::NodeHandle mppi_node, bool debug_mode, int hz, bool nodelet);
    AdamPlant(ros::NodeHandle global_node, bool debug_mode, int hz)
        : AdamPlant(global_node, global_node, debug_mode, hz, false){};

    /**
     * @brief Callback for /pose_estimate subscriber.
     */
    void poseCall(nav_msgs::Odometry pose_msg);

    /**
     * @brief Callback for recording the current servo input.
     */
    void servoCall(adam_msgs::chassisState servo_msgs);

    /**
     * @brief Callback for safe speed subscriber.
     */
    void runstopCall(adam_msgs::runstop safe_msg);

    /**
     * @brief Callback for 全局跟随轨迹
     */
    void globalTrajectoryCall(nav_msgs::Path traj_msgs);

    /**
     * @brief Publishes the controller's nominal path.
     */
    void pubPath(const ros::TimerEvent &);

    void setSolution(std::vector<float> traj, std::vector<float> controls,
                     util::EigenAlignedVector<float, 2, 7> gains,
                     ros::Time timestamp, double loop_speed);

    void setRolloutPath(std::vector<float> rolloutPath, int num_rollouts);

    void setDebugImage(cv::Mat img);

    void setTimingInfo(double poseDiff, double tickTime, double sleepTime);

    void pubTimingData(const ros::TimerEvent &);

    /**
     * @brief Publishes a control input.
     * @param steering The steering command to publish.
     * @param throttle The throttle command to publish.
     */
    void pubControl(float steering, float throttle);

    void pubStatus(const ros::TimerEvent &);

    /**
     * @brief Returns the current state of the system.
     */
    AdamPlant::FullState getState();

    /**
     * @brief Returns the current value of safe speed.
     */
    bool getRunstop();

    /**
     * @brief Returns the timestamp of the last pose callback.
     */
    ros::Time getLastPoseTime();

    /**
     * @brief 返回需要跟随的路径
     */
    nav_msgs::Path getTrajPath();

    /**
     * @brief rviz可视化
     */
    void rvizShow(float* points, int size);
    /**
     * @brief Checks the system status.
     * @return An integer specifying the status. 
     * 0: the system is operating niminally, 
     * 1: something is wrong bt noaction needs to be taken,
     * 2: vehicle should stop immediately.
     */
    int checkStatus();

    void dynRcfgCall(drive_control::PathIntegralParamsConfig &config, int lvl);

    bool hasNewDynRcfg();
    
    drive_control::PathIntegralParamsConfig getDynRcfgParams();

    virtual void displayDebugImage(const ros::TimerEvent&);

    virtual bool hasNewObstacles(){return false;}
    virtual void getObstacles(std::vector<int> &description, std::vector<float> &data){};
    
    virtual bool hasNewCostmap(){return false;}
    virtual void getCostmap(std::vector<int> &description, std::vector<float> &data){};

    virtual void shutdown();

  protected:
    int poseCount_ = 0;
    bool useFeedbackGains_ = false;
    std::atomic<bool> receivedDebugImg_;
    std::atomic<bool> debugShutdownSignal_;
    std::atomic<bool> debugShutdownSignalAcknowledged_;
    drive_control::PathIntegralParamsConfig costParams_;
    bool hasNewCostParams_ = false;

    const double TIMEOUT = 0.5; ///< Time before declaring pose/controls state

    FullState full_state_; ///< Full state of the vehicle.

    int hz_; ///< The frequency of the control publisher.

    int status_;          ///< System status
    std::string ocs_msg_; ///< Message to send to ocs.

    bool safe_speed_zero_; ///< Current value of safe speed.
    bool debug_mode_;     ///< Whether or not the system is in debug/simulation mode.
    bool activated_;      ///< Whether or not we've received an initial pose message.

    ros::Time last_pose_call_; ///< Timestam of the last pose callback.

    ros::Publisher control_pub_;         ///< Publisher of adam_msgs::chassisCommand type on topic servoCommand.
    ros::Publisher status_pub_;           ///< Publishes the status (0 good, 1 neutralm, 2 bad)) of the controller.
    ros::Publisher subscribed_pose_pub_; ///< Publisher of the subscribed pose
    ros::Publisher path_pub_;            ///< Publisher of nav_msg::Path on topic nominalPath.
    ros::Publisher timing_data_pub_;
    ros::Publisher rollout_path_pub_;    ///< Publisher of 采样路径

    ros::Subscriber pose_sub_; ///< Subscriber to /pose_estimate.
    ros::Subscriber servo_sub_;
    ros::Subscriber trajectory_sub_; ///跟随的路径

    ros::Timer pathTimer_;
    ros::Timer statusTimer_;
    ros::Timer debugImgTimer_;
    ros::Timer timingInfoTimer_;

    nav_msgs::Path traj_msg_;   ///跟随的轨迹
   

    nav_msgs::Path path_msg_;                  ///< Path message for publishing the planned path.
    geometry_msgs::Point time_delay_msg;       ///< Point message for publishing the observed delay.
    adam_msgs::pathIntegralStatus status_msg_; ///<pathIntegralStatus message for publishing mppi status
    adam_msgs::pathIntegralTiming timingData_; ///<pathIntegralStatus message for publishing mppi status

    std::vector<float> model_params_; ///< Array for holding the updated model parameters
    std::vector<float> rollout_state_;
    int NUM_ROLLOUTS = 0;
};
} // namespace drive_control
#endif /* ADAM_PLANT_H */
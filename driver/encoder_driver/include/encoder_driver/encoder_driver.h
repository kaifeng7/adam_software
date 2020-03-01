#include <ros/ros.h>
#include "gpio.h"
#include "geometry_msgs/TwistStamped.h"
#include <cmath>

namespace encoder_driver
{

class Encoder
{
  public:
    ros::Subscriber sub_endcoder_;
    Encoder(ros::NodeHandle nh, ros::NodeHandle pnh);

    void run();
    // init();
  private:
    void encoder_cal(bool encoder_value);
    void timerCallback(const ros::TimerEvent& event);

    ros::Timer timer;
    

    int gpioA_number_;
    int gpioB_number_;

    int sample_rate_;
    double param_wheel_radius_;
    double param_encoder_pulse_;

    bool first_entry_ = true;
    bool last_encoder_value_;
    int v_change_cnt_ = 0;

    //
    ros::Time start_;
    ros::Time now_;

    GPIO encoder_gpioA_;
    GPIO encoder_gpioB_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // ros::Subscriber sub_endoder;
    ros::Publisher pub_wheel_circles_;
};
} // namespace encoder_driver
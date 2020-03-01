#include <encoder_driver/encoder_driver.h>

namespace encoder_driver{
    
    Encoder::Encoder(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh){
        // nh_ = nh;
        // pnh_ = pnh;
        pub_wheel_circles_ = nh.advertise<geometry_msgs::TwistStamped>("/wheel_circles", 10);
        timer = nh.createTimer(ros::Duration(0.1), &Encoder::timerCallback, this);
        pnh_.param("gpioA_number", gpioA_number_, 296);
        pnh_.param("gpioB_number", gpioB_number_, 296);
        pnh_.param("sample_rate", sample_rate_, 2000);
        pnh_.param("wheel_radius", param_wheel_radius_, 0.075);
        pnh_.param<double>("encoder_pulse", param_encoder_pulse_, 100);
        encoder_gpioA_.set_gpio(gpioA_number_);
        // encoder_gpioB.set_gpio(gpioB_number_);
        encoder_gpioA_.edge(GPIO::EDGE_BOTH);
        // encoder_gpioB.edge(GPIO::EDGE_NONE);
        
    }

    void Encoder::timerCallback(const ros::TimerEvent& event){
        //todo add vehicle params
        // count * 2 * PI * param_wheel_radius / (param_mag_num * (now - start).toSec())
        //ROS_INFO("time callback");
        double speed = v_change_cnt_ / param_encoder_pulse_   * M_PI * param_wheel_radius_ * 10;
        geometry_msgs::TwistStamped msg_twist;
        msg_twist.header.stamp = ros::Time::now();
        msg_twist.twist.linear.x = speed;
        pub_wheel_circles_.publish(msg_twist);
        ROS_INFO("cnt=%d, pub speed=%f",v_change_cnt_, speed);
        v_change_cnt_ = 0;
    }
    void Encoder::encoder_cal(bool encoder_value)
    {
        if(first_entry_){
            //第一次进入
            first_entry_ = false;
            last_encoder_value_ = encoder_value;
        }

        if(encoder_value != last_encoder_value_){
            v_change_cnt_++;
        }
        last_encoder_value_ = encoder_value;
//        if(encoder_value) ROS_INFO("true");
  //      else ROS_INFO("false");
    }

    void Encoder::run(){
        ros::Rate loop_rate(sample_rate_);
        while(ros::ok())
        {
            encoder_gpioA_.poll(10);
            encoder_cal(encoder_gpioA_);
            ros::spinOnce();
            loop_rate.sleep();
            
        }
    }
}




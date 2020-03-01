#include <drive_control_x/path_integral/status_monitor.h>

namespace drive_control {

StatusMonitor::StatusMonitor(ros::NodeHandle nh)
{
	last_status_ = ros::Time::now();
	status_sub_ = nh.subscribe("/mppi_controller/mppiStatus", 1, &StatusMonitor::statusCallback, this);
	std::string info = "MPPI Controller";
	std::string hardwareID = "none";
	std::string portPath = "";
}

void StatusMonitor::statusCallback(adam_msgs::pathIntegralStatus msg)
{
	info_ = msg.info;
	status_ = msg.status;
	last_status_ = ros::Time::now();
}

void StatusMonitor::diagnosticStatus(const ros::TimerEvent& time)
{
	if ((double)ros::Time::now().toSec() - (double)last_status_.toSec() > TIMEOUT){
		ROS_ERROR("CONTROLLER TIMEOUT");
	}
	else if (status_ == 0){
		ROS_INFO(info_.c_str());
	}
	else if (status_ == 1){
		ROS_WARN(info_.c_str());
	}
	else if (status_ == 2){
		ROS_ERROR(info_.c_str());
	}
}

}

using namespace drive_control;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mppiStatusMonitor");
	ros::NodeHandle status_node("~");
	StatusMonitor monitor(status_node);
	ros::spin();
}
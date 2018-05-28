#include <ros/ros.h>
#include <ros/console.h>
#include <soccer_msgs/RobotGoal.h>
#include <soccer_msgs/RobotState.h>

using namespace std;
using namespace ros;
ros::NodeHandle* nh;

ros::Publisher test_trajectories;

void main() {
	ros::init(argc, argv, "soccer_hardware");
	ros::NodeHandle n;
	nh = &n;

	ros::Rate r(100); // 100 Hz

	while(ros::ok()) {

		
		r.sleep();
	}
}
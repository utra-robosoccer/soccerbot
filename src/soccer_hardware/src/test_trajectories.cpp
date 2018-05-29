#include <ros/ros.h>
#include <ros/console.h>
#include <soccer_msgs/RobotGoal.h>
#include <soccer_msgs/RobotState.h>

#include <fstream>
#include <sstream>

using namespace std;
using namespace ros;
ros::NodeHandle* nh;

ros::Publisher test_trajectories;

string TRAJECTORY_FILE = "walking.csv";

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_trajectories");
	ros::NodeHandle n;
	nh = &n;

	string filename = "../src/soccer_hardware/trajectories/" + TRAJECTORY_FILE;
	ifstream trajectory_file(filename, std::ifstream::in);
	if(!trajectory_file.is_open()) {
		cout << "Could not open file" << filename << endl;
		exit(1);
	}
	else cout << "Opened file" << endl;

	string line;

	ros::Rate r(100); // 100 Hz

	test_trajectories = n.advertise<soccer_msgs::RobotGoal>("robotGoal", 1);

	float angles[20];

	while(ros::ok()) {
		getline(trajectory_file, line);

		if(!trajectory_file) {
			trajectory_file.clear();
			trajectory_file.seekg(0, ios::beg);
			continue;
		}
		stringstream ss(line);
		cout << ss.str() << endl;
		int i = 0;
		string token;
		while(std::getline(ss, token, ',')) {
			angles[i] = stof(token);
			++i;
		}

		cout << "Sending Goal" << endl;
		soccer_msgs::RobotGoal robotGoal;
		for(int i = 0; i < 20; ++i) {
			robotGoal.trajectories[i] = angles[i];
		}
		test_trajectories.publish(robotGoal);

		r.sleep();
	}
}

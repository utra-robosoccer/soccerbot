#include <ros/ros.h>
#include <ros/console.h>
#include <soccer_msgs/RobotGoal.h>
#include <soccer_msgs/RobotState.h>
#include <std_msgs/Float64.h>

#include <fstream>
#include <sstream>

using namespace std;
using namespace ros;
ros::NodeHandle* nh;

ros::Subscriber draw_trajectories;

ros::Publisher torso_right_hip_side;
ros::Publisher right_hip_side_hip_front;
ros::Publisher right_hip_front_thigh;
ros::Publisher right_thigh_calve;
ros::Publisher right_calve_ankle;
ros::Publisher right_ankle_foot;

ros::Publisher torso_left_hip_side;
ros::Publisher left_hip_side_hip_front;
ros::Publisher left_hip_front_thigh;
ros::Publisher left_thigh_calve;
ros::Publisher left_calve_ankle;
ros::Publisher left_ankle_foot;

ros::Publisher torso_right_bicep;
ros::Publisher right_bicep_forearm;
ros::Publisher torso_left_bicep;
ros::Publisher left_bicep_forearm;
ros::Publisher torso_neck;
ros::Publisher neck_head;


void send_to_gazeboCallback(const soccer_msgs::RobotGoal::ConstPtr& msg)
{
	std_msgs::Float64 msgout;

	msgout.data = msg->trajectories[0];
	torso_right_hip_side.publish(msgout);

	msgout.data = msg->trajectories[1];
	right_hip_side_hip_front.publish(msgout);
	
	msgout.data = msg->trajectories[2];
	right_hip_front_thigh.publish(msgout);

	msgout.data = msg->trajectories[3];
	right_thigh_calve.publish(msgout);

	msgout.data = msg->trajectories[4];
	right_calve_ankle.publish(msgout);

	msgout.data = msg->trajectories[5];
	right_ankle_foot.publish(msgout);

	msgout.data = msg->trajectories[6];
	torso_left_hip_side.publish(msgout);

	msgout.data = msg->trajectories[7];
	left_hip_side_hip_front.publish(msgout);

	msgout.data = msg->trajectories[8];
	left_hip_front_thigh.publish(msgout);

	msgout.data = msg->trajectories[9];
	left_thigh_calve.publish(msgout);

	msgout.data = msg->trajectories[10];
	left_calve_ankle.publish(msgout);

	msgout.data = msg->trajectories[11];
	left_ankle_foot.publish(msgout);

	msgout.data = msg->trajectories[12];
	torso_right_bicep.publish(msgout);

	msgout.data = msg->trajectories[13];
	right_bicep_forearm.publish(msgout);

	msgout.data = msg->trajectories[14];
	torso_left_bicep.publish(msgout);

	msgout.data = msg->trajectories[15];
	left_bicep_forearm.publish(msgout);
	
	msgout.data = msg->trajectories[16];
	torso_neck.publish(msgout);

	msgout.data = msg->trajectories[17];
	neck_head.publish(msgout);
	
	cout << "Sending data" << endl;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "draw_trajectories");
	ros::NodeHandle n;
	nh = &n;

    // TODO (Nam) Move all the controls out of the /soccerbot/ namespace
    torso_right_hip_side = n.advertise<std_msgs::Float64>("torso_right_hip_side_position_controller/command", 1);
    right_hip_side_hip_front = n.advertise<std_msgs::Float64>("right_hip_side_hip_front_position_controller/command",
                                                              1);
    right_hip_front_thigh = n.advertise<std_msgs::Float64>("right_hip_front_thigh_position_controller/command", 1);
    right_thigh_calve = n.advertise<std_msgs::Float64>("right_thigh_calve_position_controller/command", 1);
    right_calve_ankle = n.advertise<std_msgs::Float64>("right_calve_ankle_position_controller/command", 1);
    right_ankle_foot = n.advertise<std_msgs::Float64>("right_ankle_foot_position_controller/command", 1);

    torso_left_hip_side = n.advertise<std_msgs::Float64>("torso_left_hip_side_position_controller/command", 1);
    left_hip_side_hip_front = n.advertise<std_msgs::Float64>("left_hip_side_hip_front_position_controller/command", 1);
    left_hip_front_thigh = n.advertise<std_msgs::Float64>("left_hip_front_thigh_position_controller/command", 1);
    left_thigh_calve = n.advertise<std_msgs::Float64>("left_thigh_calve_position_controller/command", 1);
    left_calve_ankle = n.advertise<std_msgs::Float64>("left_calve_ankle_position_controller/command", 1);
    left_ankle_foot = n.advertise<std_msgs::Float64>("left_ankle_foot_position_controller/command", 1);

    torso_right_bicep = n.advertise<std_msgs::Float64>("torso_right_bicep_position_controller/command", 1);
    right_bicep_forearm = n.advertise<std_msgs::Float64>("right_bicep_forearm_position_controller/command", 1);
    torso_left_bicep = n.advertise<std_msgs::Float64>("torso_left_bicep_position_controller/command", 1);
    left_bicep_forearm = n.advertise<std_msgs::Float64>("left_bicep_forearm_position_controller/command", 1);

    torso_neck = n.advertise<std_msgs::Float64>("torso_neck_position_controller/command", 1);
    neck_head = n.advertise<std_msgs::Float64>("neck_head_position_controller/command", 1);
	
	ros::Rate r(100); // 100 Hz

	draw_trajectories = n.subscribe("robotGoal", 1, send_to_gazeboCallback);

	while(ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}
}

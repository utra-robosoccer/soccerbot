#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <termio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <soccer_msgs/RobotGoal.h>
#include <soccer_msgs/RobotState.h>

using namespace std;
using namespace ros;
ros::NodeHandle* nh;
int fd;

ros::Subscriber send_to_robot;
ros::Publisher receive_from_robot;

typedef struct robotGoal {
	uint32_t startSeq;
	uint32_t id;
	char message[80];
	uint32_t endSeq;
} RobotGoal;

typedef struct robotstate {
	uint32_t id;
	char message[80];
} RobotState;

RobotState robotState;
RobotGoal robotGoal;

int open_port(void) {
	int fd; // file description for the serial port

	fd = open("/dev/ttyTHS2",
			O_RDWR | O_NONBLOCK);
	if (fd == -1)
		fd = open("/dev/ttyACM2", O_RDWR | O_NONBLOCK);
	if (fd == -1)
		fd = open("/dev/ttyACM1", O_RDWR | O_NONBLOCK);
	if (fd == -1)
		fd = open("/dev/ttyACM0", O_RDWR | O_NONBLOCK);
	if (fd == -1)
		fd = open("/dev/ttyUSB0", O_RDWR | O_NONBLOCK);
	if (fd == -1) {
		cout << "open_port: Unable to open /dev/ttyACM0. \n" << endl;
		exit(1);
	}

	tcflush(fd, TCIOFLUSH);

	cout << "Opened Port" << endl;

	return (fd);
} //open_port

int configure_port(int fd) {
	struct termios port_settings;     // structure to store the port settings in

	cfsetispeed(&port_settings, B1000000);    // set baud rates
	cfsetospeed(&port_settings, B1000000);

	port_settings.c_cflag = ((port_settings.c_cflag & ~CSIZE) | CS8);

	port_settings.c_cflag |= CLOCAL | CREAD;


	port_settings.c_cflag &= ~(PARENB | PARODD);    // set no parity, stop bits, data bits

	port_settings.c_cflag &= ~CRTSCTS;

	port_settings.c_cflag &= ~CSTOPB;
	

	port_settings.c_iflag = IGNBRK;

	port_settings.c_iflag &= ~(IXON | IXOFF | IXANY);

	port_settings.c_lflag = 0;
	port_settings.c_oflag = 0;

	port_settings.c_cc[VTIME]=1;
	port_settings.c_cc[VMIN]=1;

	// port_settings.c_iflag &= ~IGNCR;
	// port_settings.c_iflag &= ~ICRNL;
	// port_settings.c_iflag &= ~INLCR;
	// port_settings.c_iflag &= ~INPCK;
	// port_settings.c_iflag &= ~ISTRIP;

	port_settings.c_lflag &= ~(ECHONL|NOFLSH);

	int mcs = 0;
    ioctl(fd, TIOCMGET, &mcs);
    mcs |= TIOCM_RTS;
	ioctl(fd, TIOCMSET, &mcs);

	port_settings.c_cflag &= ~CRTSCTS;


	tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port

	cout << "Configured Port" << endl;
	return (fd);
}

int max_buf_size = 128;

int totalSentPackets = 0;
int totalFailedPackets = 0;

void receive_loop() {
	unsigned char robotStateData[sizeof(RobotState)];
	unsigned char *robotStateDataPtr = robotStateData;
	int totalBytesRead = 0;

	int startseqcount = 0;

	unsigned char m_buf[max_buf_size];

	while (1) {
		ros::spinOnce();

		int bytesRead = read(fd, m_buf, max_buf_size);

		if(bytesRead < 0) continue;

		for (int x = 0; x < bytesRead; ++x) {
			// Start reading
			if(startseqcount == 4) {
				*robotStateDataPtr = m_buf[x];
				robotStateDataPtr++;
				totalBytesRead++;
				if(totalBytesRead == sizeof(RobotState))
					break;
			}
			else {
				if(m_buf[x] == 255)
					startseqcount++;
				else
					startseqcount = 0;
			}
		}

		if(totalBytesRead == sizeof(RobotState))
			break;
	}

	memset(&robotState, 0, sizeof(RobotState));
	memcpy(&robotState, robotStateData, sizeof(RobotState));

	ROS_INFO("Recieved data");

	soccer_msgs::RobotState state;

	for(int i = 0; i < 20; ++i) {
		char* ptr = &robotState.message[i * 4];
		memcpy(&state.joint_angles[i], ptr, 4);
	}

	for(int i = 0; i < 80; ++i) {
		printf("%x ", (unsigned) robotState.message[i] & 0xff);
	}
	cout << endl;

	receive_from_robot.publish(state);
}

int id = 0;

void send_to_robotCallback(const soccer_msgs::RobotGoal::ConstPtr& msg)
{
	for(int i = 0; i < 20; ++i) {
		float f = msg->trajectories[i];
		char* ptr = &robotGoal.message[i * 4];
		memcpy(ptr, &f, 4);
	}

	robotGoal.id = ++id;

	ROS_INFO("Sending To Robot");
	for(int i = 0; i < 80; ++i) {
		printf("%x ", (unsigned) robotGoal.message[i] & 0xff);
	}
	cout << endl;

	char buf[sizeof(RobotGoal)];
	memcpy(buf, &robotGoal, sizeof(RobotGoal));
	for(int i = 0; i < sizeof(RobotGoal); ++i) {
		unsigned s[1];
    	s[0] = buf[i];
		write(fd, s, 1);
	}
}

int main(int argc, char **argv) {

	//establishConnection();
	ros::init(argc, argv, "soccer_hardware");
	ros::NodeHandle n;
	nh = &n;

	ros::Rate r(1); // 100 Hz

	fd = open_port();
	configure_port(fd);

	robotGoal.startSeq = UINT32_MAX;
	robotGoal.endSeq = 0;

	send_to_robot = n.subscribe("robotGoal", 1, send_to_robotCallback);
	receive_from_robot = n.advertise<soccer_msgs::RobotState>("robotState", 1);

	while(ros::ok()) {
		receive_loop();

		r.sleep();
	}
}


/* TEAM_COMMUNICATION NODE:
 * team_communication_server:
 * 		receives local_model.msg sent by every robots' personal_model node.
 * 		and merge them together with the robot's local_model. then send
 * 		it as team_data.msg to team_update_model node
 * team-communciation_client:
 * 		receives local_model.msg of the robot from personal_model node.
 * 		then send it to the other robots' team_communication_server node.
 *
 * **2
 * 		temporary using local_model.msg saved in msg directory.
 * 		after personal_model.msg implemented in filtering package, switch to use that one.
 * */

/* SERVER NODE */

#include <ros/ros.h>
#include <iostream>
#include <cstring>
#include <sys/types.h> //change it to cpp library
#include <sys/socket.h>
#include <sys/time.h>
#include <netdb.h>
#include <iostream>
#include <string>
#include <std_msgs/String.h>
#include <team_communication/local_model.h>
#include <team_communication/team_local_model.h>
#include <team_communication/team_data.h>

#define ROBOT_ID_MIN 0
#define PLAYER_NUM 5
#define	LOCAL_MODEL_PORT 3636

using namespace std;

ros::Publisher pub;
static team_communication::local_model team_models[PLAYER_NUM];

static void parser_rcv(char* buff, team_communication::team_data& msg)
{
	uint8_t robotID = 0;
	uint8_t min = ROBOT_ID_MIN;
	uint8_t max = PLAYER_NUM;
	team_communication::local_model model;
	
	if( NULL == buff )
		goto exit;
		
	robotID = (uint8_t)buff[0];
	printf("%u", robotID);
	if( min > robotID || max <= robotID )
	{
		cout << "id is out of range" << endl;
		goto exit;
	}
		
	
	memcpy(&model, &buff[1], sizeof(model));
	
	cout << "model: " ;
	printf("%u, ", model.x);
	printf("%u, ", model.y);
	printf("%u\n", model.z);
	
	//update model info
	team_models[robotID] = model;	
	
	//prepare ros msg
	for(int i = 0 ; i < PLAYER_NUM ; i++ )
	{
		msg.team_models[i] = team_models[i];
	}
	
exit:

	return;
	
}

static void models_init(void)
{
	for(int i = 0 ; i < PLAYER_NUM ; i++ )
	{
		team_models[i].x = 0;
		team_models[i].y = 0;
		team_models[i].z = 0;
	}
}
	

int main(int argc, char **argv)
{
	//ros setup
	ros::init(argc, argv, "team_communication_server");
	ros::NodeHandle n;

	//init team_models
	models_init();
	
	// increase the capacity of each if needed
	pub = n.advertise<team_communication::team_data>("team_data", 1);
	
	
	//broadcast server setup
	struct sockaddr_in my_addr;
    struct timeval tv;
	int s = 0;
	int bc = 1;
	tv.tv_sec = 1;
	tv.tv_usec = 0;
	
	if((s=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP )) < 0)
	{
		printf("failed to assign socket\n");
		goto exit;
	}
	
	setsockopt(s, SOL_SOCKET,SO_BROADCAST, &bc, sizeof(bc));
	setsockopt(s, SOL_SOCKET,SO_REUSEADDR, &bc, sizeof(bc));
	setsockopt(s, SOL_SOCKET,SO_RCVTIMEO, (const char*)&tv,sizeof(struct timeval)); //change later
	
	memset(&my_addr,0,sizeof(my_addr));
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(LOCAL_MODEL_PORT);
	my_addr.sin_addr.s_addr = INADDR_ANY;
	
	if(bind(s, (sockaddr*)&my_addr,sizeof(sockaddr)) < 0)
	{
		printf("failed to bind socket\n");
		goto exit;
	}

	while(ros::ok())
	{
		char buff[1024] = "\0";
		team_communication::team_data msg;
		unsigned int len = sizeof(my_addr);
		
		recvfrom(s, buff, sizeof(buff),0,(sockaddr*)&my_addr, &len);
		parser_rcv(buff, msg);
		pub.publish(msg);

		ros::spinOnce();
		//sleep();
	}
	
exit:

	return 0;
}

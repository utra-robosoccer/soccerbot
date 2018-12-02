
/* soccer_team_communication NODE:
 * soccer_team_communication_server:
 * 		receives local_model.msg sent by every robots' personal_model node.
 * 		and merge them together with the robot's local_model. then send
 * 		it as team_data.msg to team_update_model node
 * team-communciation_client:
 * 		receives local_model.msg of the robot from personal_model node.
 * 		then send it to the other robots' soccer_team_communication_server node.
 *
 * *1
 * 		temporary leave the callback functions for strategy.msg/robot_state.msg/global_model blank !!!
 * **2
 * 		temporary using local_model.msg saved in msg directory.
 * 		after personal_model.msg implemented in filtering package, switch to use that one.
 */

/* CLIENT NODE */

#include <ros/ros.h>
#include <iostream>
#include <cstring>
#include <sys/types.h> //change it to cpp library
#include <sys/socket.h>
#include <sys/time.h>
#include <netdb.h>
#include <std_msgs/Int32.h>   //temporary
#include <std_msgs/String.h>
#include <soccer_team_communication/local_model.h>
#include <soccer_team_communication/team_local_model.h>

//#include <behaviour/strategy.h>
//#include <robot_control/robot_state.h>
//#include <filtering/global_model.h>

#define ROBOT_ID 0    					//HARD CODED
#define LOCAL_MODEL_PORT  3636			//modify if needed

using namespace std;

soccer_team_communication::local_model personal_model;

static void callbackLocalModel(const soccer_team_communication::local_model::ConstPtr& msg)
{
	printf("called\n");
	personal_model.x = msg->x;
	personal_model.y = msg->y;
	personal_model.z = msg->z;
	cout << "rcv: " << msg->x << msg->y << msg->z << endl;
}

/* TEMPORARALY LEAVE THEM BLANK */
static void callbackStrategy(const std_msgs::Int32::ConstPtr& msg)  //behaviour::strategy::ConstPtr& msg
{
	//empty
}

static void callbackRobotState(const std_msgs::Int32::ConstPtr& msg)  //robot_control::robot_state::ConstPtr& msg
{
	//empty
}

static void callbackGlobalModel(const std_msgs::Int32::ConstPtr& msg)  //filtering::global_model::ConstPtr& msg
{
	//empty
}
/* TEMPORARY ENDS */

static void parser_send(char* buff)
{
	uint8_t id = ROBOT_ID;
	
	if( NULL == buff )
		goto exit;
	
	memcpy(buff, &id, sizeof(id));
	memcpy(buff+sizeof(id), &personal_model, sizeof(personal_model));

exit:

	return;
}

static void personal_model_init(void)
{
	personal_model.x = 0;
	personal_model.y = 0;
	personal_model.z = 0;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "soccer_team_communication_client");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    personal_model_init();

    //modify topic name later
    ros::Subscriber sub = n.subscribe("local_model", 10, callbackLocalModel);
    ros::Subscriber sub2 = n.subscribe("strategy", 10, callbackStrategy);
    ros::Subscriber sub3 = n.subscribe("robot_state", 10, callbackRobotState);
    ros::Subscriber sub4 = n.subscribe("global_model", 10, callbackGlobalModel);

	//setup sockets
	struct sockaddr_in my_addr;
	int s,bc;
	
	if((s=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP )) < 0 )
	{
		printf("failed to assign socket\n");
		goto exit;
	}
	
	setsockopt(s, SOL_SOCKET,SO_BROADCAST, &bc, sizeof(bc));
	memset(&my_addr,0,sizeof(my_addr));
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(LOCAL_MODEL_PORT);
	my_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
	
	
    while(ros::ok())
    {
		char buff[1024] = "\0";
		
		parser_send(buff);
		sendto(s, buff, sizeof(buff),0,(sockaddr*)&my_addr, sizeof(my_addr));
		
		printf("sent:\n roboid:%u, model_x: %u, model_y: %u, model_z: %u\n", buff[0],buff[1],buff[2],buff[3]);
		
    	ros::spinOnce();
    	loop_rate.sleep();
    }
    
 exit:
	
	return 0;
}

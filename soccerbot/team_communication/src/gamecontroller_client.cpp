
#include <ros/ros.h>
#include <cstring>
#include <sys/types.h> //change it ?
#include <sys/socket.h>
#include <sys/time.h>
#include <netdb.h>
#include <iostream>
#include <string>
#include <std_msgs/Int32.h>   //temporary
#include <team_communication/game_state.h>
#include <team_communication/game_stateRes.h>
//#include <robot_control/pausedbool.h>

//port
#define GAMECONTROLLER_DATA_PORT 3838
#define GAMECONTROLLER_RETURN_PORT 3939

//for sanity check
#define GAME_TYPE_NUM 3
#define GAME_STATE_NUM 5
#define SECONDARY_GAME_STATE_NUM 7
#define TEAM_COLOR_NUM	10
#define SPL_PENALTY_NUM 10
#define PENALTY_SUBSTITUTE 14
#define PENALTY_MANUAL 15
#define HL_PENALTY_BALL_MANIPULATION 30
#define HL_PENALTY_SERVICE	35
#define PLAYER_RES_NUM 3

#define UNDEFINED 255

using namespace std;

ros::Publisher pub;
uint8_t protocol_ver = 0; 
int robotInfoSize = sizeof(team_communication::robotInfo);
int teamInfoSize = sizeof(team_communication::teamInfo); 

/* temporary parametes*/
uint8_t team_ = 1;
uint8_t player_ = 9;
uint8_t message_ = 2;
/* temporary ends */

enum sanity_check_type{
	SANITY_CHECK_GAME_TYPE,
	SANITY_CHECK_STATE,
	SANITY_CHECK_SECONDARY_STATE,
	SANITY_CHECK_TEAM_COLOR,
	SANITY_CHECK_PENALTY,
	SANITY_CHECK_PLAYER_MSG
};

static void sanity_check( char buff, char& rtn_val, sanity_check_type type )
{	
	uint8_t undefined = UNDEFINED;
	int32_t val_in_int = (int32_t)buff;
	
	
	switch( type )
	{
		case SANITY_CHECK_GAME_TYPE:
			if( val_in_int < GAME_TYPE_NUM )
				rtn_val = buff;
			else
				rtn_val = undefined;	
			break;
		case SANITY_CHECK_STATE:
			if( val_in_int < GAME_STATE_NUM )
				rtn_val = buff;
			else
				rtn_val = undefined;
			break;
		case SANITY_CHECK_SECONDARY_STATE:
			if( val_in_int < SECONDARY_GAME_STATE_NUM )
				rtn_val = buff;
			else
				rtn_val = undefined;
			break;
		case SANITY_CHECK_TEAM_COLOR:
			if( val_in_int < TEAM_COLOR_NUM )
				rtn_val = buff;
			else
				rtn_val = undefined;
			break;
		case SANITY_CHECK_PENALTY:
			if( val_in_int < SPL_PENALTY_NUM ||
				val_in_int == PENALTY_MANUAL ||
				val_in_int == PENALTY_SUBSTITUTE||
				(val_in_int <= HL_PENALTY_SERVICE && 
				val_in_int >= HL_PENALTY_BALL_MANIPULATION) )
			{
				rtn_val = buff;
			}
			else
			{
				rtn_val = buff;
			}
			break;
		case SANITY_CHECK_PLAYER_MSG:
			if( val_in_int < PLAYER_RES_NUM )
				rtn_val = buff;
			else
				rtn_val = undefined;
			break;
		default:
			rtn_val = buff;
			break;
	}
}

static void parser_rcv(const char* buff, team_communication::game_state& msg)
{	
	char val = UNDEFINED;
	
	memcpy(&msg.header, buff, sizeof(msg.header));
	memcpy(&msg.protocol_version, &buff[4], sizeof(msg.protocol_version));
	memcpy(&msg.packetNum, &buff[6], sizeof(msg.packetNum));
	memcpy(&msg.playersPerTeam, &buff[7], sizeof(msg.playersPerTeam));
	
	sanity_check( buff[8], val, SANITY_CHECK_GAME_TYPE);
	memcpy(&msg.gameType, &val, sizeof(msg.gameType));
	
	sanity_check( buff[9], val, SANITY_CHECK_STATE);
	memcpy(&msg.state, &val, sizeof(msg.state));
	
	memcpy(&msg.firsthalf, &buff[10], sizeof(msg.firsthalf));
	memcpy(&msg.kickoffTeam, &buff[11], sizeof(msg.kickoffTeam));
	
	sanity_check( buff[12], val, SANITY_CHECK_SECONDARY_STATE);
	memcpy(&msg.secondaryState, &val, sizeof(msg.secondaryState));
		
	memcpy(&msg.secondaryStateInfo, &buff[13], sizeof(msg.secondaryStateInfo) );
	memcpy(&msg.dropInTeam, &buff[17], sizeof(msg.dropInTeam));
	memcpy(&msg.dropInTime, &buff[18], sizeof(msg.dropInTime));
	memcpy(&msg.secsRemaining, &buff[20], sizeof(msg.secsRemaining));
	memcpy(&msg.secondaryTime, &buff[22], sizeof(msg.secondaryTime));
	
	//team info
	for( int i = 0 ; i < 2 ; i++ )
	{
		int team_offset = i*teamInfoSize;
		memcpy(&msg.Teams[i].teamNum, &buff[24 + team_offset], sizeof(team_communication::teamInfo::teamNum));
		
		sanity_check( buff[25], val, SANITY_CHECK_TEAM_COLOR);
		memcpy(&msg.Teams[i].teamColour, &val, sizeof(team_communication::teamInfo::teamColour));
		
		memcpy(&msg.Teams[i].score, &buff[26 + team_offset], sizeof(team_communication::teamInfo::score));
		memcpy(&msg.Teams[i].penaltyShot, &buff[27 + team_offset], sizeof(team_communication::teamInfo::penaltyShot));
		memcpy(&msg.Teams[i].singleShots, &buff[28 + team_offset], sizeof(team_communication::teamInfo::singleShots));
		memcpy(&msg.Teams[i].coachSequence, &buff[30 + team_offset], sizeof(team_communication::teamInfo::coachSequence));
		memcpy(&msg.Teams[i].coachMessage, &buff[31 + team_offset], sizeof(team_communication::teamInfo::coachMessage));
		
		//robot info
		sanity_check( buff[284 + team_offset], val, SANITY_CHECK_PENALTY);
		memcpy(&msg.Teams[i].coach.penalty, &val, sizeof(team_communication::robotInfo::penalty));
		
		memcpy(&msg.Teams[i].coach.secsTillUnpenalised, &buff[285 + team_offset], sizeof(team_communication::robotInfo::secsTillUnpenalised));
		memcpy(&msg.Teams[i].coach.yellowCardCount, &buff[286 + team_offset], sizeof(team_communication::robotInfo::yellowCardCount));
		memcpy(&msg.Teams[i].coach.redCardCount, &buff[287 + team_offset], sizeof(team_communication::robotInfo::redCardCount));
		
		for( int j = 0; j < 11; j++ )
		{
			int robot_offset = j*robotInfoSize + team_offset;
			
			sanity_check( buff[288 + robot_offset], val, SANITY_CHECK_PENALTY);
			memcpy(&msg.Teams[i].players[j].penalty, &val, sizeof(team_communication::robotInfo::penalty));
			memcpy(&msg.Teams[i].players[j].secsTillUnpenalised, &buff[289 + robot_offset], sizeof(team_communication::robotInfo::secsTillUnpenalised));
			memcpy(&msg.Teams[i].players[j].yellowCardCount, &buff[290 + robot_offset], sizeof(team_communication::robotInfo::yellowCardCount));
			memcpy(&msg.Teams[i].players[j].redCardCount, &buff[291 + robot_offset], sizeof(team_communication::robotInfo::redCardCount));
		}
	}
	
}


static void parser_res(char* tmp_buff)
{	
	char val = UNDEFINED;
	int32_t offset = 0;
	char header_[5] = "RGrt";
	
	memcpy(tmp_buff, header_, sizeof(team_communication::game_stateRes::header));
	offset = sizeof(team_communication::game_stateRes::header);
	
	memcpy(tmp_buff+offset, &protocol_ver, sizeof(team_communication::game_stateRes::protocol_version));
	offset += sizeof(team_communication::game_stateRes::protocol_version);
	
	memcpy(tmp_buff+offset, &team_, sizeof(team_communication::game_stateRes::team));
	offset += sizeof(team_communication::game_stateRes::team);
	
	memcpy(tmp_buff+offset, &player_, sizeof(team_communication::game_stateRes::player));
	offset += sizeof(team_communication::game_stateRes::player);

	sanity_check( message_, val, SANITY_CHECK_PLAYER_MSG);
	memcpy(tmp_buff+offset, &val, sizeof(team_communication::game_stateRes::message));
		
}

static void callbackFunction(const std_msgs::Int32::ConstPtr& msg)  
{
	/* !!!! type of msg is temporary set to Int32 !!!! */
	
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
	my_addr.sin_port = htons(GAMECONTROLLER_RETURN_PORT);
	my_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
	
	//send
	char buff_send[1024];
	parser_res(buff_send);
	sendto(s, buff_send, sizeof(buff_send)+1,0,(sockaddr*)&my_addr, sizeof(my_addr));
	printf("sent: %s\n",buff_send);
	
exit:
	return;

}

int main(int argc, char **argv) {
	
	//setup ros
    ros::init(argc, argv, "gamecontroller_client");
    ros::NodeHandle n;
    pub = n.advertise<team_communication::game_state>("game_state", 10);  //adjust capacity if needed
    ros::Subscriber sub = n.subscribe("pausedbool", 10, callbackFunction); 
    
    //setup socket etc
    struct sockaddr_in my_addr;
    struct timeval tv;
	int s = 0;
	int bc = 1;
	tv.tv_sec = 2;
	tv.tv_usec = 0;
	
	
	if((s=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP )) < 0)
	{
		printf("failed to assign socket\n");
		goto exit;
	}
	
	setsockopt(s, SOL_SOCKET,SO_BROADCAST, &bc, sizeof(bc));
	setsockopt(s, SOL_SOCKET,SO_REUSEADDR, &bc, sizeof(bc));
	setsockopt(s, SOL_SOCKET,SO_RCVTIMEO, (const char*)&tv,sizeof(struct timeval)); //modify later
	
	memset(&my_addr,0,sizeof(my_addr));
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(GAMECONTROLLER_DATA_PORT);
	my_addr.sin_addr.s_addr = INADDR_ANY;
	
	if(bind(s, (sockaddr*)&my_addr,sizeof(sockaddr)) < 0)
	{
		printf("failed to bind socket\n");
		goto exit;
	}
	
    
    while( ros::ok() ) 
    {
		char buff[1024] = "\0";
		team_communication::game_state msg;
		unsigned int len = sizeof(my_addr);
		
		recvfrom(s, buff, sizeof(buff)-1,0,(sockaddr*)&my_addr, &len);
		parser_rcv(buff, msg);
		protocol_ver = msg.protocol_version;
		pub.publish(msg);
		printf("%u",msg.gameType);
		cout << msg.header[0] << msg.header[1] << msg.header[2] << msg.header[3] << endl;
		
		//deal with subscribing msg
		ros::spinOnce(); 
	}

exit:

	return 0;
}

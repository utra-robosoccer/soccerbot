#!/usr/bin/python

#team_communication_server test

import socket
import rospy
from team_communication.msg import team_local_model
from team_communication.msg import team_data

SENDER_PORT = 3737
id_ = 2

def callback(rcv_data):
	print "rcved %s" %rcv_data.team_models[id_]

def test():
	#ros
	rospy.init_node('test_tc_server', anonymous=True)
	
	#input
	data_ = team_local_model()
	data_.roboID = id_
	data_.model.x = 100
	data_.model.y = 100
	data_.model.z = 100
	
	print "sent:"
	print "roboid:%d, model_x:%u, model_y:%u, model_z:%u" %(data_.roboID, data_.model.x, data_.model.y, data_.model.z)
	
	#send broadcast
	data = chr(data_.roboID) + chr(data_.model.x) + chr(data_.model.y) + chr(data_.model.z)
	sct = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sct.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) 
	sct.sendto(data,('<broadcast>',SENDER_PORT))
	sct.close()
	
	#receive ros message
	rospy.Subscriber('team_data', team_data, callback)
	rospy.spin()
		
if __name__ == "__main__":
	test()

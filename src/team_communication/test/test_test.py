#!/usr/bin/python

#team_communication_server test

import rospy
from team_communication.msg import local_model

SENDER_PORT = 3737
id_ = 2

def callback(rcv_data):
	print "rcved: %s" %rcv_data

def test():
	#ros
	rospy.init_node('test', anonymous=True)
	
	#receive ros message
	rospy.Subscriber('local_model', local_model, callback)
	rospy.spin()
		
if __name__ == "__main__":
	test()

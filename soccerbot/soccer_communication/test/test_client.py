#!/usr/bin/python

#team_communication_client node test

import socket
import rospy
from team_communication.msg import local_model


RCV_PORT = 3636

def test():
	#ros
	rospy.init_node('test_tc_client', anonymous=True)
	
	#data
	data = local_model()
	data.x  = 100
	data.y = 100
	data.z = 100
	
	
	#publish
	r = rospy.Rate(10)
	count = 0
	while (not rospy.is_shutdown() and count < 5):
		pub = rospy.Publisher('local_model', local_model, queue_size=100)
		rospy.loginfo(str)
		pub.publish(data)
		print "sent model: %u, %u, %u" %(data.x, data.y,data.z)
		count += 1
		r.sleep()
	
	
	#rcv bc
	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	s.bind(('',RCV_PORT))
	rcv = s.recvfrom(1024)
	
	print rcv
	#print "rcved model: %u, %u, %u" %(rcv[1], rcv[2], rcv[3])
	
if __name__ == "__main__":
	test()

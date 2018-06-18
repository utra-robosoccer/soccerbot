#!/usr/bin/env python

#interation test for team_communication client/server nodes

import rospy
from team_communication.msg import local_model
from team_communication.msg import team_local_model
from team_communication.msg import team_data

id_ = 0

def callback(rcv_data):
	print rcv_data.team_models[id_]

def test():
	#ros
	rospy.init_node('test_tc_integrate', anonymous=True)
	
	#input
	data_ = local_model()
	data_.x  = 100
	data_.y = 100
	data_.z = 100
	
	#send ros msg
	r = rospy.Rate(10)
	count = 0
	while (not rospy.is_shutdown() and count < 5):
		pub = rospy.Publisher('local_model', local_model, queue_size=100)
		pub.publish(data_)
		print "sent model: %u, %u, %u" %(data_.x, data_.y,data_.z)
		count += 1
		r.sleep()
	
	#rcv ros msg
	rospy.Subscriber('team_data', team_data, callback)
	rospy.spin()

if __name__ == "__main__":
	test()

#!/usr/bin/env python
import rospy
from environment.srv import *
from environment.msg import *
from std_msgs.msg import String
import time

pub_action = actor_action_msg()
pub_action.action = [0.0,0.0,0.0]
pub_action.reset = 0


def handle_action(req):
    print("handling request: ")
    print(req.action)
    pub_action.action = req.action.action
    pub_action.reset = req.action.reset
    return action_srvResponse(1) 

def state_server():

    rospy.init_node("env_action_server")
    s = rospy.Service("env_action_service", action_srv, handle_action)
    print("Environment Action Server Ready")
    pub = rospy.Publisher("action_publisher",actor_action_msg,queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print("published action")
        rospy.loginfo(pub_action)
        pub.publish(pub_action)
        rate.sleep()
    

if __name__=="__main__":

    try:
        state_server()
    except rospy.ROSInterruptException:
        pass 

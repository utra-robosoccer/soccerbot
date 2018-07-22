#!/usr/bin/env python
import rospy
from environment.srv import *
from environment.msg import *

state = env_state_msg() 
state.state = []
state.done = 0

def state_callback(state_msg):
    print("observing state")
    state.state = state_msg.state
    state.done = state_msg.done

def handle_state(req):
    print("reporting state: ")
    print(state)
    response = env_state_msg()
    if req.request_state == 1:
        response = state
    return state_srvResponse(response) 

def state_server():

    rospy.init_node("env_state_server")
    s = rospy.Service("env_state_service", state_srv, handle_state)
    rospy.Subscriber("state_publisher",env_state_msg,state_callback)
    print("Environment State Server Ready")
    rospy.spin()

if __name__=="__main__":
    state_server()

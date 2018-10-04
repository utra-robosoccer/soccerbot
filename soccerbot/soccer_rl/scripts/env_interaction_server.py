#!/usr/bin/env python
import rospy
from environment.srv import *
from environment.msg import *
import time

def calculate_reward(state):
    print("calculating reward")
    return 5

def action_client(action):
    print("Requesting action")
    rospy.wait_for_service('env_action_service') 
    try:
        take_action = rospy.ServiceProxy('env_action_service',action_srv)
        response = take_action(action)
        if response.complete == 1:
            return 1 
    except rospy.ServiceException, e:
        print("Service call failed: {}".format(e))
        return None

def state_client():
    print("querying state")
    rospy.wait_for_service('env_state_service')
    try:
        get_state = rospy.ServiceProxy('env_state_service',state_srv)
        response = get_state(1)
        current_pos = response.state.state
        done = response.state.done
        return current_pos,done 
    except rospy.ServiceException, e:
        print("Service call failed: {}".format(e))
        return [],None

def handle_interaction(req):

    print("Requested action is: {}".format(req.action))

    int_response = env_observation_msg()
    int_response.state = []
    int_response.reward = 0
    int_response.done = 0
    int_response.info = ""

    confirmed = action_client(req.action)
    if confirmed == None:
        int_response.info = "Action Request Failed"
        print(int_response.info)
        return int_response
    time.sleep(0.2)
    current_state,done = state_client()
    if done == None:
         int_response.info = "State Query Failed" 
         print(int_response.info)
         return int_response
    reward = calculate_reward(current_state)
    int_response.state = current_state
    int_response.reward = reward
    int_response.done = done
    int_response.info = "Success"
    print("handle success, response state: {}, response reward: {}, response done: {}, response info: {}".format(int_response.state,int_response.reward,int_response.done,int_response.info))
    print("--------------------------------")
    return interaction_srvResponse(int_response) 

def interaction_server():   
    rospy.init_node("env_int_server")
    s = rospy.Service('env_int_service', interaction_srv, handle_interaction)
    print("Environment Interaction Server Ready")
    rospy.spin()

if __name__=="__main__":

    interaction_server()

#!/usr/bin/env python
import sys
import rospy
from environment.srv import *
from environment.msg import *

def interaction_client(action):
    rospy.wait_for_service('env_int_service')
    try:
        interact = rospy.ServiceProxy('env_int_service',interaction_srv)
        response = interact(action)
        state = response.observation.state
        reward = response.observation.reward
        done = response.observation.done
        info = response.observation.info
        return state,reward,done,info
    except rospy.ServiceException, e:
        print("Service call failed: {}".format(e))
        return [],0,0,""

def main():
    action_vals = [1.0,2.0,3.0]
    action = actor_action_msg()
    action.action = action_vals
    action.reset = 0
    while True:
        state,reward,done,info = interaction_client(action)
        print("state is: {}, reward is: {}, done is: {}, info is: {}".format(state,reward,done,info))
        print("--------")
        
if __name__ == "__main__":
    main()

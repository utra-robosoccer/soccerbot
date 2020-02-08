#!/usr/bin/env python
import rospy
from std_msgs.msg import String

# Define the publisher and subscribers here

class Action:
    def __init__(self):
        pass

    def execute(self):
        # ros.publish geometry,Pose2D where to go
        # ros.publish rviz debug pose 2D

        # ros,publish soccer_trajectories string for getupfront getupback
        pass

class State:
    robots = {}
    ball = {}

    def update(self):
        # Reads all tf transformations and updates the information and the status from nam
        # robot[1]["position"] = (2,3) # geometry_msgs::Pose2D
        # robot[1]["status"] = Status.FallenBack

        # Update the location of the ball
        # ball["position"] = (2,3)

        pass

    def successors(self):
        # Retrieve list of actions in future (robot can move in radius 1m, and front half circle (slices) 10 degrees
        # If ball is within 1m. robot can also move directly to the ball, slightly to the left and the right because of foot
        # If robot.Status = Fallen, then the only thing you can do is get back up (front and back)
        pass

    def value(self):
        # for every robot. Add them up
        # Return the heuristic value of the state
        # if fallen, h = 0
        # if standing h = 100
        # distance to ball
        pass

    def __init__(self):
        pass


def main():
    rospy.init_node('soccer_strategy', anonymous=True)
    rate = rospy.Rate(0.05)  # 0.1 hz

    state = State
    while not rospy.is_shutdown():
        state.update()

        value = 0
        for (action, sucessor) in state.successors():
            if sucessor.value > value:
                best_state = sucessor

        action.execute()

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg

# Define the publisher and subscribers here
ball_position = rospy.Publisher('ball_position', geometry_msgs.msg.Point, queue_size=1)
robot_pose = rospy.Publisher('amcl_pose', geometry_msgs.msg.Pose, queue_size=1)
tfBuffer = None
listener = None


class Action:
    def __init__(self):
        pass

    def execute(self):
        # ros.publish geometry,Pose2D where to go
        # ros.publish rviz debug pose 2D

        # ros,publish soccer_trajectories string for getupfront getupback
        pass


class State:


    def update(self):
        # Reads all tf transformations and updates the information and the status from nam
        # robot[1]["position"] = (2,3) # geometry_msgs::Pose2D
        # robot[1]["status"] = Status.FallenBack

        # Update the location of the ball
        # ball["position"] = (2,3)

        has_ball = False

        try:
            ball_pose = tfBuffer.lookup_transform('ball', 'base_footprint', rospy.Time(0))
            has_ball = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            pass

        last_pose = rospy.Time.now() - ball_pose.header.stamp;

        if has_ball and (last_pose < rospy.Duration(1)):
            position = geometry_msgs.msg.Point()

            position.x = ball_pose.transform.translation.x
            position.y = ball_pose.transform.translation.y
            position.z = 0

            self.ball["position"] = position




        pass

    def successors(self):  # using pose array for all states
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
        self.robots = {}
        self.ball = {}
        pass


def main():
    global tfBuffer
    global listener

    rospy.init_node('soccer_strategy', anonymous=True)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)  # 0.05 hz

    state = State()

    while not rospy.is_shutdown():
        state.update()

        value = 0
        '''for (action, sucessor) in state.successors():
            if sucessor.value > value:
                best_state = sucessor

        action.execute()'''

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

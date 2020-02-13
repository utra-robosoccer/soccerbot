#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import PoseArray
import math

# Define the publisher and subscribers here


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

    def update(self):  # finished but not scalable
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
        # only 180 deg front of robot at 10 degree interval in direction of ball
        # Retrieve list of actions in future (robot can move in radius 1m, and front half circle (slices) 10 degrees
        # If ball is within 1m. robot can also move directly to the ball, slightly to the left and the right because of foot
        # If robot.Status = Fallen, then the only thing you can do is get back up (front and back)

        pose_array = PoseArray()
        pose_array.header.frame_id = 'world'
        pose_array.header.stamp = rospy.Time.now()

        dist = self.distance((self.ball["position"]))

        if self.robot_pose[1]["status"] == 1:
            pass
        # if ball not in 1m
        elif dist > 1.0:
            for i in range(0, 190, 10):
                future_pose = geometry_msgs.msg.Pose()
                if i <= 90:
                    future_pose.position.y = (self.robot_pose[1]["position"].y - math.cos(math.radians(i)))
                else:
                    future_pose.position.y = (self.robot_pose[1]["position"].y + math.cos(math.radians(i)))
                future_pose.position.x = (self.robot_pose[1]["position"].x - math.sin(math.radians(i)))
                #pose_array.poses.append((future_pose, ))
            pass
        # if ball is in 1m but to far x and y
        else:
            # right in front of the ball
            if abs((self.ball["position"].y - self.robot_pose[1]["position"].y)) <= 0.1:
                # goes forward
                pass
            # to the side of the ball

            pass

        pass

    def value(self):
        # for every robot. Add them up
        # Return the heuristic value of the state
        # if fallen, h = 0
        # if standing h = 100
        # distance to ball
        pass

    def robot_pose_callback(self, data):
        self.robot_pose[1]["position"] = data.position
        pass

    def distance(self, position):
        dist = (position.x ** 2 + (position.y) ** 2) ** (1 / 2)
        return dist
        pass

    def __init__(self):
        self.robot_pose = rospy.Subscriber("amcl_pose", geometry_msgs.msg.Pose, self.robot_pose_callback)
        # self.ball_position = rospy.Publisher('ball_position', geometry_msgs.msg.Point, queue_size=1)

        position = geometry_msgs.msg.Point()
        position.x = 0.0
        position.y = 0.0
        position.z = 0.0

        self.robots = {1: {'position': position, 'status': 0}}
        self.ball = {'position': position}
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

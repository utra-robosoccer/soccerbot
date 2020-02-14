#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import PoseArray
from tf.transformations import quaternion_from_euler
import math
from enum import Enum


class Status(Enum):
    standing = 0
    walking = 1
    fallen_back = 2
    fallen_forward = 3
    getting_up = 4
    kicking = 5


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
        ball_pose = geometry_msgs.msg.TransformStamped()
        ball_pose.header.stamp = rospy.Time.now()

        try:
            ball_pose = tfBuffer.lookup_transform('ball', 'base_footprint', rospy.Time(0))
            has_ball = True
            self.has_ball_once = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            pass

        last_pose = rospy.Time.now() - ball_pose.header.stamp;

        if has_ball and (last_pose < rospy.Duration(1)):
            position = geometry_msgs.msg.PoseWithCovarianceStamped()

            position.pose.pose.position.x = ball_pose.transform.translation.x
            position.pose.pose.position.y = ball_pose.transform.translation.y
            position.pose.pose.position.z = 0

            position.pose.pose.orientation.x = 0.0
            position.pose.pose.orientation.y = 0.0
            position.pose.pose.orientation.z = 0.0
            position.pose.pose.orientation.w = 1.0

            self.ball["position"] = position
            self.successors()

        pass

    def successors(self):  # using pose array for all states
        # only 180 deg front of robot at 10 degree interval in direction of ball
        # Retrieve list of actions in future (robot can move in radius 1m, and front half circle (slices) 10 degrees
        # If ball is within 1m. robot can also move directly to the ball, slightly to the left and the right because of foot
        # If robot.Status = Fallen, then the only thing you can do is get back up (front and back)
        if not self.has_ball_once:
            return
        self.successor['pose_array'].poses = []
        self.successor['pose_array'].header.frame_id = 'world'
        self.successor['pose_array'].header.stamp = rospy.Time.now()

        self.successor['cost'] = []
        self.successor['status'] = []
        future_pose = geometry_msgs.msg.Pose()
        q = [0, 0, 0, 1]
        future_pose.orientation.x = q[0]  # maybe add a real orientation based off of quaternions
        future_pose.orientation.y = q[1]
        future_pose.orientation.z = q[2]
        future_pose.orientation.w = q[3]

        dist = self.distance(self.ball["position"].pose.pose.position.x, self.ball["position"].pose.pose.position.y)
        if self.robots[1]["status"] == Status.walking:
            return
        elif self.robots[1]["status"] == Status.fallen_back:
            self.successor['status'].append(Status.fallen_back)
            self.successor['cost'].append(0)
            self.value(self.successor)
            pass

        elif self.robots[1]["status"] == Status.fallen_forward:
            self.successor['status'].append(Status.fallen_forward)
            self.successor['cost'].append(0)
            self.value(self.successor)
            pass
        # if ball not in 1m
        elif self.robots[1]["status"] == Status.standing:
            if abs((self.ball["position"].pose.pose.position.y - self.robots[1]["position"].pose.pose.position.y)) <= 0.05:
                # goes forward and kick
                self.successor['status'].append(Status.kicking)
                self.successor['cost'].append(0)
                self.value(self.successor)
                pass
            if dist > 1.0:
                for i in range(0, 190, 10):

                    if i > 90:
                        future_pose.position.y = (
                                self.robots[1]["position"].pose.pose.position.y - math.cos(math.radians(i - 90)))
                        future_pose.position.x = (
                                self.robots[1]["position"].pose.pose.position.x + math.sin(math.radians(i - 90)))
                    else:
                        future_pose.position.y = (
                                self.robots[1]["position"].pose.pose.position.y + math.cos(math.radians(i)))
                        future_pose.position.x = (
                                self.robots[1]["position"].pose.pose.position.x + math.sin(math.radians(i)))
                    future_pose.position.z = 0.0
                    self.successor['pose_array'].poses.append(future_pose)
                    self.successor['cost'].append(0)
                    self.successor['status'].append(Status.walking)
                self.future_pose.publish(self.successor['pose_array'])
                self.value(self.successor)

            elif dist < 1.0:
                for i in range(0, 190, 10):

                    if i > 90:
                        future_pose.position.y = (
                                self.robots[1]["position"].pose.pose.position.y - (
                                math.cos(math.radians(i - 90)) / dist))
                        future_pose.position.x = (
                                self.robots[1]["position"].pose.pose.position.x + (
                                math.sin(math.radians(i - 90)) / dist))
                    else:
                        future_pose.position.y = (
                                self.robots[1]["position"].pose.pose.position.y + (math.cos(math.radians(i)) / dist))
                        future_pose.position.x = (
                                self.robots[1]["position"].pose.pose.position.x + (math.sin(math.radians(i)) / dist))
                    future_pose.position.z = 0.0
                    self.successor['pose_array'].poses.append(future_pose)
                    self.successor['cost'].append(0)
                    self.successor['status'].append(Status.walking)
                self.future_pose.publish(self.successor['pose_array'])
                self.value(self.successor)


        pass

    def value(self, next_state):
        # for every robot. Add them up
        # Return the heuristic value of the state
        # if fallen, h = 0
        # if standing h = 100
        # distance to ball
        if not next_state['status']:
            return
        h = 0
        for i in range(0, len(next_state['status'])):

            if next_state['status'][i] == Status.fallen_back:
                h = 0

                pass

            elif next_state['status'][i] == Status.fallen_forward:
                h = 0
                pass

            elif next_state['status'][i] == Status.standing:
                h += 100
                # ball position in robot frame + current robot frame = ball in world  => ball in world - robot new position
                x = self.distance(
                    self.ball["position"].pose.pose.position.x + self.robots[1]['position'].pose.pose.position.x -
                    next_state[i]['pose_array'].position.x)
                y = self.distance(
                    self.ball["position"].pose.pose.position.y + self.robots[1]['position'].pose.pose.position.y -
                    next_state[i]['pose_array'].position.y)

                h += self.distance(x, y)
                pass
            elif next_state['status'][i] == Status.kicking:
                h += 10

            self.successor['cost'][i] = h

    def robot_pose_callback(self, data):
        self.robots[1]["position"] = data
        pass

    def distance(self, x, y):
        dist = (x ** 2 + y ** 2) ** (1 / 2)
        return dist
        pass

    def __init__(self):
        self.robot_pose = rospy.Subscriber("amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped,
                                           self.robot_pose_callback)
        self.future_pose = rospy.Publisher('successor_states', geometry_msgs.msg.PoseArray, queue_size=1)

        position = geometry_msgs.msg.PoseWithCovarianceStamped()

        position.pose.pose.position.x = 0.0
        position.pose.pose.position.y = 0.0
        position.pose.pose.position.z = 0.0

        position.pose.pose.orientation.x = 0.0
        position.pose.pose.orientation.y = 0.0
        position.pose.pose.orientation.z = 0.0
        position.pose.pose.orientation.w = 1.0

        self.robots = {1: {'position': position, 'status': Status.standing}}
        self.ball = {'position': position}

        pose_array = PoseArray()
        cost = []
        status = []
        self.has_ball_once = False
        self.successor = {'pose_array': pose_array, 'cost': cost, 'status': status}
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

        value = 2**16
        best_state = PoseArray()
        state.successors()
        for i in range(0,len(state.successor['status'])):
            if state.successor['cost'][i] < value:
                value = state.successor['cost'][i]
                best_state = state.successor['pose_array']
        #action.execute()

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

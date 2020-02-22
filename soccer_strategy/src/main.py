#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import PoseArray
from tf.transformations import quaternion_from_euler
import math
from enum import Enum
from std_msgs.msg import String


class Status(Enum):
    standing = 0
    walking = 1
    fallen_back = 2
    fallen_forward = 3
    kicking = 4


# Define the publisher and subscribers here


tfBuffer = None
listener = None


class Action:
    def __init__(self):
        self.get_up = rospy.Publisher('command', String, queue_size=1, latch=True)
        self.move = rospy.Publisher('goal', geometry_msgs.msg.Pose, queue_size=1, latch=True)
        pass

    def execute(self, state, robots):
        # ros.publish geometry,Pose2D where to go
        # ros.publish rviz debug pose 2D

        # ros,publish soccer_trajectories string for getupfront getupback

        if state["status"] == Status.fallen_forward:
            msg = String()
            msg.data = "getupfront"
            self.get_up.publish(msg)
            while robots[1]['status'] != Status.standing:
                rospy.sleep(0.1)
            pass
        elif state["status"] == Status.fallen_back:
            msg = String()
            msg.data = "getupfront"
            self.get_up.publish(msg)
            while robots[1]['status'] != Status.standing:
                rospy.sleep(0.1)
            pass
        elif state["status"] == Status.standing:
            msg = geometry_msgs.msg.Pose()
            msg.position = state["state"].position
            msg.orientation = state["state"].orientation
            self.move.publish(msg)
            robots[1]['status'] = Status.walking
            while robots[1]['status'] != Status.standing:
                rospy.sleep(0.1)
            pass

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
        self.successor['pose_array'] = PoseArray()
        self.successor['pose_array'].header.frame_id = 'world'
        self.successor['pose_array'].header.stamp = rospy.Time.now()
        self.successor['cost'] = []
        self.successor['status'] = []

        dist = self.distance(self.ball["position"].pose.pose.position.x, self.ball["position"].pose.pose.position.y)
        tmp = geometry_msgs.msg.Pose()
        tmp.orientation = (0, 0, 0, 1)
        tmp.position = (0, 0, 0, 0)

        if self.robots[1]["status"] == Status.walking:
            return
        elif self.robots[1]["status"] == Status.fallen_back:
            self.successor['pose_array'].poses.append(tmp)
            self.successor['status'].append(Status.fallen_back)
            self.successor['cost'].append(0)

            pass

        elif self.robots[1]["status"] == Status.fallen_forward:
            self.successor['pose_array'].poses.append(tmp)
            self.successor['status'].append(Status.fallen_forward)
            self.successor['cost'].append(0)

            pass
        elif self.robots[1]["status"] == Status.standing:
            if abs((self.ball["position"].pose.pose.position.y - self.robots[1][
                "position"].pose.pose.position.y)) <= 0.0:  # arbitray distance 0.0 so that the point array is generated
                self.successor['status'].append(Status.kicking)
                self.successor['cost'].append(10)

            if dist >= 1.0:
                dist = 1.0

            for i in range(0, 190, 10):
                future_state = geometry_msgs.msg.Pose()
                q = [0, 0, 0, 1]
                future_state.orientation.x = q[0]  # maybe add a real orientation based off of quaternions
                future_state.orientation.y = q[1]
                future_state.orientation.z = q[2]
                future_state.orientation.w = q[3]

                if i > 90:
                    future_state.position.y = (self.robots[1]["position"].pose.pose.position.y - (
                            math.cos(math.radians(i - 90)) / dist))
                    future_state.position.x = (self.robots[1]["position"].pose.pose.position.x + (
                            math.sin(math.radians(i - 90)) / dist))
                else:
                    future_state.position.y = (
                            self.robots[1]["position"].pose.pose.position.y + (math.cos(math.radians(i)) / dist))
                    future_state.position.x = (
                            self.robots[1]["position"].pose.pose.position.x + (math.sin(math.radians(i)) / dist))
                future_state.position.z = 0.0

                self.successor['pose_array'].poses.append(future_state)
                self.value(future_state.position.x, future_state.position.y)
                self.successor['cost'].append(0)
                self.successor['status'].append(Status.walking)

            self.future_pose.publish(self.successor['pose_array'])

        pass

    def value(self, x, y):
        # for every robot. Add them up
        # Return the heuristic value of the state
        # if fallen, h = 0
        # if standing h = 100
        # distance to ball
        h = 100
        # ball position in robot frame + current robot frame = ball in world  => ball in world - robot new position
        x_2 = self.ball["position"].pose.pose.position.x + self.robots[1]['position'].pose.pose.position.x - x
        y_2 = self.ball["position"].pose.pose.position.y + self.robots[1]['position'].pose.pose.position.y - y
        h += self.distance(x_2, y_2)
        return h

    def robot_pose_callback(self, data):
        self.robots[1]["position"] = data
        pass

    def robot_state_callback(self, data):
        if data.data == 'fell_front':
            self.robots[1]["status"] = Status.fallen_forward
        elif data.data == 'fell_back':
            self.robots[1]["status"] = Status.fallen_back
        elif data.data == 'standing':
            self.robots[1]["status"] = Status.standing
        pass

    def distance(self, x, y):
        dist = (x ** 2 + y ** 2) ** (1 / 2)
        return dist
        pass

    def __init__(self):
        self.robot_pose = rospy.Subscriber("amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped,
                                           self.robot_pose_callback)
        self.future_pose = rospy.Publisher("successor_states", geometry_msgs.msg.PoseArray, queue_size=1)
        self.robot_state = rospy.Subscriber("fall_state", String, self.robot_state_callback)

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
    act = Action()
    while not rospy.is_shutdown():
        state.update()

        value = 2 ** 16
        best_state = {'state': geometry_msgs.msg.Pose, 'status': Status.standing}

        state.successors()
        if len(state.successor['status']) >= 1:
            for i in range(0, len(state.successor['status'])):
                if state.successor['cost'][i] < value:
                    value = state.successor['cost'][i]
                    best_state['state'] = state.successor['pose_array'].poses[i]
                    best_state['status'] = state.successor['status'][i]
            act.execute(best_state, state.robots)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

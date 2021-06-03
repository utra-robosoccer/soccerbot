from robot import Robot
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, Empty, Bool
import numpy as np
import math
import tf.transformations
from sensor_msgs.msg import Imu


class RobotRos(Robot):
    def __init__(self, team, role, status, robot_name):
        self.robot_pose_sub = rospy.Subscriber('/' + robot_name + "/amcl_pose",
                                               PoseWithCovarianceStamped,
                                               self.robot_pose_callback)
        self.ball_pose_sub = rospy.Subscriber('/' + robot_name + "/ball",
                                              PoseWithCovarianceStamped,
                                              self.ball_pose_callback)
        self.imu_sub = rospy.Subscriber('/' + robot_name + "/imu_filtered", Imu, self.imu_callback)
        self.goal_publisher = rospy.Publisher('/' + robot_name + "/goal", PoseStamped, queue_size=1)
        self.trajectory_publisher = rospy.Publisher('/' + robot_name + "/command", String, queue_size=1)
        self.terminate_walking_publisher = rospy.Publisher('/'+ robot_name + "/terminate_walking", Empty, queue_size=1)
        self.completed_walking_subscriber = rospy.Subscriber('/'+ robot_name + "/completed_walking", Empty, self.completed_walking_callback)
        self.completed_trajectory_subscriber = rospy.Subscriber('/'+ robot_name + "/trajectory_complete", Bool, self.completed_trajectory_subscriber)
        self.start_walking_publisher = rospy.Publisher('/'+ robot_name + "/start_walking", Empty, queue_size=1)

        self.team = team
        self.role = role
        self.status = status
        self.position = np.array([0.0, 0.0, 0])
        self.goal_position = np.array([0.0, 0.0, 0])
        self.ball_position = np.array([0.0, 0.0])
        self.robot_name = robot_name
        self.max_kick_speed = 2
        self.previous_status = Robot.Status.READY

        # for static trajectories
        self.trajectory_complete = True

    def robot_pose_callback(self, data):
        quaternion = (
            data.pose.pose.orientation.w,
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.position = np.array([-data.pose.pose.position.y, data.pose.pose.position.x, -euler[0] - math.pi/2])
        pass

    def ball_pose_callback(self, data):
        self.ball_position = np.array([-data.pose.pose.position.y, data.pose.pose.position.x])
        pass

    def completed_walking_callback(self, data):
        self.status = Robot.Status.READY
        print("Completed Walking")
        pass

    def completed_trajectory_subscriber(self, data):
        self.trajectory_complete = data.data

    def set_navigation_position(self, position):
        super(RobotRos, self).set_navigation_position(position)
        print("Sending Robot " + self.robot_name + " to position" + str(position))
        p = PoseStamped()
        p.header.stamp = rospy.get_rostime()
        p.header.frame_id = "world"
        p.pose.position.x = position[1]
        p.pose.position.y = -position[0]
        p.pose.position.z = 0
        angle_fixed = position[2]
        #print(angle_fixed)
        q = tf.transformations.quaternion_about_axis(angle_fixed, (0, 0, 1))
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        self.goal_publisher.publish(p)
        if self.status == Robot.Status.WALKING:
            self.start_walking_publisher.publish()

    def imu_callback(self, msg):
        angle_threshold = 1  # in radian
        q = msg.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([q.w, q.x, q.y, q.z])

        if self.status == Robot.Status.WALKING or self.status == Robot.Status.READY:
            # We want to publish once on state transition
            if pitch > angle_threshold:
                print("fall back triggered")
                self.status = Robot.Status.FALLEN_BACK

            if pitch < -angle_threshold:
                print("fall front triggered")
                self.status = Robot.Status.FALLEN_FRONT
        pass


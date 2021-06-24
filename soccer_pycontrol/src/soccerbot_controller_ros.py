import numpy as np
import tf

from soccerbot_controller import *
import rospy
from soccerbot_ros import SoccerbotRos
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose
from std_msgs.msg import Empty, Bool


class SoccerbotControllerRos(SoccerbotController):

    def __init__(self):
        if rospy.get_param('ENABLE_PYBULLET'):
            if rospy.get_param('COMPETITION'):
                pb.connect(pb.DIRECT)
            else:
                pb.connect(pb.GUI)
            pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
            pb.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0, cameraPitch=0,
                                          cameraTargetPosition=[0, 0, 0.25])
            pb.setGravity(0, 0, -9.81)

        self.soccerbot = SoccerbotRos(Transformation(), useFixedBase=False)
        self.ramp = Ramp("plane.urdf", (0, 0, 0), (0, 0, 0), lateralFriction=0.9, spinningFriction=0.9,
                         rollingFriction=0.0)

        self.position_subscriber = rospy.Subscriber("goal", PoseStamped, self.goal_callback)
        self.robot_position_subscriber = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped,
                                                          self.robot_pose_callback)
        self.terminate_walk_subscriber = rospy.Subscriber("terminate_walking", Empty, self.terminate_walk_callback)
        self.completed_walk_publisher = rospy.Publisher("completed_walking", Empty, queue_size=1)
        self.finish_trajectory = rospy.Subscriber('trajectory_complete', Bool, self.trajectory_callback, queue_size=1)
        self.fixed_trajectory_running = False
        self.goal = PoseStamped()
        self.robot_pose = None
        self.new_goal = self.goal
        self.terminate_walk = False

        self.tf_listener = tf.TransformListener()

    def trajectory_callback(self, msg):
        self.soccerbot.reset_imus()
        self.soccerbot.ready()
        self.fixed_trajectory_running = not msg.data
        if msg.data:
            self.terminate_walk = False
        pass

    def robot_pose_callback(self, pose: PoseWithCovarianceStamped):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('world', os.environ["ROS_NAMESPACE"] + '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        self.robot_pose = PoseStamped()
        self.robot_pose.pose.position.x = trans[0]
        self.robot_pose.pose.position.y = trans[1]

        self.robot_pose.pose.orientation.x = rot[0]
        self.robot_pose.pose.orientation.y = rot[1]
        self.robot_pose.pose.orientation.z = rot[2]
        self.robot_pose.pose.orientation.w = rot[3]

    def goal_callback(self, pose: PoseStamped):
        self.new_goal = pose
        pass

    def terminate_walk_callback(self, val):
        rospy.logwarn("Terminating Walk Requested")
        self.soccerbot.ready()
        self.soccerbot.publishAngles()
        self.terminate_walk = True

    def pose_to_transformation(self, pose: Pose) -> Transformation:
        t = Transformation([pose.position.x, pose.position.y, pose.position.z],
                           [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        return t

    def transformation_to_pose(self, trans: Transformation) -> PoseStamped:
        t = PoseStamped()
        t.pose.position.x = trans.get_position()[0]
        t.pose.position.y = trans.get_position()[1]
        t.pose.position.z = trans.get_position()[2]

        t.pose.orientation.x = trans.get_orientation()[0]
        t.pose.orientation.y = trans.get_orientation()[1]
        t.pose.orientation.z = trans.get_orientation()[2]
        t.pose.orientation.w = trans.get_orientation()[3]
        return t

    def ready(self):
        pass

    def robot_pose_callback_nothing(self, pose: PoseWithCovarianceStamped):
        pass

    def setPose(self, pose: Transformation):

        resetPublisher = rospy.Publisher("/reset_robot", Pose, queue_size=1, latch=True)
        initialPosePublisher = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
        pose_stamped = self.transformation_to_pose(pose)
        resetPublisher.publish(pose_stamped.pose)
        self.robot_pose_callback = lambda pose : pose
        self.robot_pose = pose_stamped
        sleep(3)
        p = PoseWithCovarianceStamped()
        p.pose.pose = pose_stamped.pose
        initialPosePublisher.publish(p)
        pass

    def setGoal(self, goal: Transformation):
        self.goal_callback(self.transformation_to_pose(goal))

    def wait(self, steps: int):
        for i in range(steps):
            rospy.sleep(SoccerbotController.PYBULLET_STEP)
            if rospy.get_param('ENABLE_PYBULLET'):
                pb.stepSimulation()

    def run(self, stop_on_completed_trajectory=False):
        t = 0
        r = rospy.Rate(1 / SoccerbotController.PYBULLET_STEP)
        stable_count = 30
        self.soccerbot.ready()
        self.soccerbot.reset_imus()

        while not rospy.is_shutdown():
            if self.robot_pose is not None and self.new_goal != self.goal:
                rospy.loginfo("Recieved New Goal")
                self.soccerbot.ready()  # TODO Cancel walking
                self.soccerbot.reset_imus()
                self.soccerbot.setPose(self.pose_to_transformation(self.robot_pose.pose))
                self.goal = self.new_goal
                self.soccerbot.setGoal(self.pose_to_transformation(self.goal.pose))
                print("robotpose ", self.robot_pose.pose)
                # print("robotpose_trans ", self.pose_to_transformation(self.robot_pose.pose.pose))
                print("goalpose ", self.goal.pose)
                # print("goalpose_trans ", self.pose_to_transformation(self.goal.pose))
                self.soccerbot.robot_path.show()
                self.soccerbot.publishPath()
                self.terminate_walk = False
                t = -5

            if self.terminate_walk:
                if self.soccerbot.robot_path is not None:
                    t = self.soccerbot.robot_path.duration() + 1

            if self.soccerbot.robot_path is not None and self.soccerbot.current_step_time <= t <= self.soccerbot.robot_path.duration():
                self.soccerbot.stepPath(t, verbose=False)
                self.soccerbot.apply_imu_feedback(t, self.soccerbot.get_imu())

                if rospy.get_param('ENABLE_PYBULLET'):
                    forces = self.soccerbot.apply_foot_pressure_sensor_feedback(self.ramp.plane)
                    pb.setJointMotorControlArray(bodyIndex=self.soccerbot.body, controlMode=pb.POSITION_CONTROL,
                                                 jointIndices=list(range(0, 20, 1)),
                                                 targetPositions=self.soccerbot.get_angles(),
                                                 forces=forces
                                                 )
                self.soccerbot.current_step_time = self.soccerbot.current_step_time + self.soccerbot.robot_path.step_size
                self.soccerbot.publishOdometry()

            if self.soccerbot.robot_path is not None and t <= self.soccerbot.robot_path.duration() < t + SoccerbotController.PYBULLET_STEP:
                rospy.loginfo("Completed Walk")
                e = Empty()
                self.completed_walk_publisher.publish(e)

            if self.soccerbot.robot_path is None or t > self.soccerbot.robot_path.duration():
                self.soccerbot.apply_head_rotation()

            if t < 0:
                if self.soccerbot.imu_ready:
                    pitch = self.soccerbot.apply_imu_feedback_standing(self.soccerbot.get_imu())
                    rospy.logwarn_throttle(0.3, "Moving to desired pitch: " + str(pitch - self.soccerbot.DESIRED_PITCH_2))
                    if abs(pitch - self.soccerbot.DESIRED_PITCH_2) < 0.025:
                        stable_count = stable_count - 1
                        if stable_count == 0:
                            t = 0
                    else:
                        stable_count = 30

            # Post walk stabilization
            if self.soccerbot.robot_path is not None and t > self.soccerbot.robot_path.duration():
                if self.soccerbot.imu_ready and not self.fixed_trajectory_running:
                    self.soccerbot.apply_imu_feedback_standing(self.soccerbot.get_imu())

            if self.soccerbot.robot_path is None and self.soccerbot.imu_ready and not self.fixed_trajectory_running:
                # print("here")
                self.soccerbot.apply_imu_feedback_standing(self.soccerbot.get_imu())

            if stop_on_completed_trajectory:
                if (self.soccerbot.robot_path is not None and t > self.soccerbot.robot_path.duration()) or self.fixed_trajectory_running:
                    break

            if not self.terminate_walk and not self.fixed_trajectory_running:
                self.soccerbot.publishAngles()  # Disable to stop walking
                if rospy.get_param('ENABLE_PYBULLET'):
                    pb.stepSimulation()

            t = t + SoccerbotController.PYBULLET_STEP
            r.sleep()

    def correct_goal_pose(self):
        pass

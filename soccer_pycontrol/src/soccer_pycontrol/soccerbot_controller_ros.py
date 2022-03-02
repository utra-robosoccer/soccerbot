import tf
from rospy import ROSInterruptException

from soccer_pycontrol.soccerbot_controller import *
import rospy
from soccer_pycontrol.soccerbot_ros import SoccerbotRos
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, PoseArray
from std_msgs.msg import Empty, Bool
from soccer_msgs.msg import RobotState
import copy


class SoccerbotControllerRos(SoccerbotController):

    def __init__(self):
        pb.connect(pb.DIRECT)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        pb.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0, cameraPitch=0,
                                      cameraTargetPosition=[0, 0, 0.25])
        pb.setGravity(0, 0, -9.81)

        self.soccerbot = SoccerbotRos(Transformation(), useFixedBase=False)
        self.ramp = Ramp("plane.urdf", (0, 0, 0), (0, 0, 0), lateralFriction=0.9, spinningFriction=0.9,
                         rollingFriction=0.0)

        self.position_subscriber = rospy.Subscriber("goal", PoseStamped, self.goal_callback)

        self.completed_walk_publisher = rospy.Publisher("action_complete", Empty, queue_size=1)
        self.goal = PoseStamped()
        self.robot_pose = None
        self.new_goal = self.goal
        self.terminated = None

        self.tf_listener = tf.TransformListener()

    def update_robot_pose(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('world', os.environ["ROS_NAMESPACE"] + '/base_footprint',
                                                            rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False
        self.robot_pose = PoseStamped()
        self.robot_pose.pose.position.x = trans[0]
        self.robot_pose.pose.position.y = trans[1]

        self.robot_pose.pose.orientation.x = rot[0]
        self.robot_pose.pose.orientation.y = rot[1]
        self.robot_pose.pose.orientation.z = rot[2]
        self.robot_pose.pose.orientation.w = rot[3]

        return True

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

    def setPose(self, pose: Transformation):
        [r, p, y] = pose.get_orientation_euler()
        q_new = Transformation.get_quaternion_from_euler([r, 0, 0])
        pose.set_orientation(q_new)

        resetPublisher = rospy.Publisher("/reset_robot", Pose, queue_size=1, latch=True)
        initialPosePublisher = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
        pose_stamped = self.transformation_to_pose(pose)
        resetPublisher.publish(pose_stamped.pose)
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

    def goal_callback(self, pose: PoseStamped):

        # Update existing path
        # Note the code is here instead of run because the computation will disturb the main thread. Main thread should still be running
        if self.soccerbot.robot_path is not None:
            print("Updating New Goal")
            start = time.time()
            goal_position = self.pose_to_transformation(pose.pose)
            self.soccerbot.addTorsoHeight(goal_position)
            self.new_path = copy.deepcopy(self.soccerbot.robot_path)

            try:
                self.t_new_path = self.new_path.dynamicallyUpdateGoalPosition(self.t, goal_position)
            except Exception as ex:
                print(ex)
                return

            end = time.time()

            self.soccerbot.publishPath(self.new_path)
            print("New Goal Updated, Time Taken: ", end - start)
            pass
        self.new_goal = pose

    def run(self, stop_on_completed_trajectory=False):
        self.t = 0
        r = rospy.Rate(1 / SoccerbotController.PYBULLET_STEP)
        stable_count = 5

        while self.soccerbot.robot_state.status == RobotState.STATUS_DISCONNECTED:
            try:
                r.sleep()
            except ROSInterruptException:
                exit(0)

        self.soccerbot.ready()
        self.soccerbot.reset_imus()
        time_now = 0

        while not rospy.is_shutdown():
            if self.soccerbot.robot_state.status in [RobotState.STATUS_TERMINATING_WALK, RobotState.STATUS_FALLEN_FRONT,
                                                     RobotState.STATUS_FALLEN_BACK, RobotState.STATUS_FALLEN_SIDE,
                                                     RobotState.STATUS_PENALTY, RobotState.STATUS_TRAJECTORY_IN_PROGRESS]:
                if not self.terminated:
                    rospy.loginfo("Terminating Walk at time " + str(self.t))
                    self.soccerbot.robot_path.terminateWalk(self.t)
                    self.terminated = True

                self.goal = self.new_goal
                r.sleep()
                continue

            # New goal added
            if self.new_goal != self.goal and self.soccerbot.robot_path is None:
                pose_updated = self.update_robot_pose()
                if not pose_updated:
                    rospy.logwarn_throttle(1, "Unable to get Robot Pose")
                    r.sleep()
                    continue

                rospy.loginfo("Received New Goal")
                time_now = rospy.Time.now()

                self.goal = self.new_goal
                self.soccerbot.reset_imus()
                self.soccerbot.ready()
                self.soccerbot.setPose(self.pose_to_transformation(self.robot_pose.pose))
                self.soccerbot.createPathToGoal(self.pose_to_transformation(self.goal.pose))
                self.t = -0.5
                print("Start Pose: ", self.robot_pose.pose)
                print("End Pose: ", self.goal.pose)
                # self.soccerbot.robot_path.show()
                self.soccerbot.publishPath()
                self.terminated = False

            # Existing goal updated
            if self.new_goal != self.goal and self.soccerbot.robot_path is not None and self.t > self.t_new_path:
                print("Updating Existing Goal and Path" + str(self.new_goal))
                self.goal = self.new_goal
                self.soccerbot.robot_path = self.new_path

            # IMU feedback while walking
            if self.soccerbot.robot_path is not None and self.soccerbot.current_step_time <= self.t <= self.soccerbot.robot_path.duration():
                self.soccerbot.stepPath(self.t, verbose=False)
                if self.soccerbot.imu_ready:
                    self.soccerbot.apply_imu_feedback(self.t, self.soccerbot.get_imu())

                forces = self.soccerbot.apply_foot_pressure_sensor_feedback(self.ramp.plane)
                self.soccerbot.current_step_time = self.soccerbot.current_step_time + self.soccerbot.robot_path.step_size
                self.soccerbot.publishOdometry()

            # Walk completed
            if self.soccerbot.robot_path is not None and self.t <= self.soccerbot.robot_path.duration() < self.t + SoccerbotController.PYBULLET_STEP and not self.terminated:
                walk_time = (rospy.Time.now().secs + (rospy.Time.now().nsecs / 100000000) - (
                            time_now.secs + (time_now.nsecs / 100000000)))
                rospy.loginfo("Completed Walk, Took: " + str(walk_time))
                e = Empty()
                self.completed_walk_publisher.publish(e)

            if self.soccerbot.robot_path is None or self.t > self.soccerbot.robot_path.duration():
                self.soccerbot.apply_head_rotation()
                self.soccerbot.robot_path = None
                pass

            if self.t < 0:
                if self.soccerbot.imu_ready:
                    pitch = self.soccerbot.apply_imu_feedback_standing(self.soccerbot.get_imu())
                    rospy.logwarn_throttle(0.3, "Performing prewalk stabilization, distance to desired pitch: " + str(
                        pitch - self.soccerbot.DESIRED_PITCH_2))
                    if abs(pitch - self.soccerbot.DESIRED_PITCH_2) < 0.025:
                        stable_count = stable_count - 1
                        if stable_count == 0:
                            t = 0
                    else:
                        stable_count = 5

            # Post walk stabilization
            if self.soccerbot.robot_path is not None and self.t > self.soccerbot.robot_path.duration():
                rospy.loginfo_throttle_identical(1, "Performing post stabilization")
                if self.soccerbot.imu_ready:
                    self.soccerbot.apply_imu_feedback_standing(self.soccerbot.get_imu())
                    pass

            if stop_on_completed_trajectory:
                if (self.soccerbot.robot_path is not None and self.t > self.soccerbot.robot_path.duration()):
                    rospy.loginfo(1, "Trajectory Stopped")
                    break

            self.soccerbot.publishAngles()  # Disable to stop walking

            self.t = self.t + SoccerbotController.PYBULLET_STEP

            try:
                r.sleep()
            except rospy.exceptions.ROSInterruptException:
                break

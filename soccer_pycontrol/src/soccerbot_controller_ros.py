from soccerbot_controller import *
import rospy
from soccerbot_ros import SoccerbotRos
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class SoccerbotControllerRos(SoccerbotController):

    def __init__(self):
        pb.connect(pb.GUI)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        pb.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[0, 0, 0.25])
        pb.setGravity(0, 0, -9.81)

        self.soccerbot = SoccerbotRos(Transformation(), useFixedBase=False)
        self.ramp = Ramp("plane.urdf", (0, 0, 0), (0, 0, 0), lateralFriction=0.9, spinningFriction=0.9, rollingFriction=0.0)

        self.position_subscriber = rospy.Subscriber("goal", PoseStamped, self.goal_callback)
        self.robot_position_subscriber = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.robot_pose_callback)
        self.goal = PoseStamped()
        self.new_goal = self.goal

    def robot_pose_callback(self, pose):
        self.robot_pose = pose
        pass

    def goal_callback(self, pose):
        self.new_goal = pose
        pass

    def pose_to_transformation(self, pose):
        t = Transformation([pose.position.x, pose.position.y, pose.position.z],
                           [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        return t

    def run(self):
        t = 0
        r = rospy.Rate(1/SoccerbotController.PYBULLET_STEP)
        while not rospy.is_shutdown():
            if self.new_goal != self.goal:
                self.soccerbot.setPose(self.pose_to_transformation(self.robot_pose.pose.pose))
                self.wait(500)

                self.goal = self.new_goal
                self.soccerbot.ready() # TODO Cancel walking
                self.soccerbot.publishAngles()
                self.wait(150)

                # Reset robot position and goal
                self.soccerbot.setGoal(self.pose_to_transformation(self.goal.pose))
                t = 0

            if self.soccerbot.robot_path is not None and self.soccerbot.current_step_time <= t <= self.soccerbot.robot_path.duration():
                self.soccerbot.stepPath(t, verbose=True)
                pb.setJointMotorControlArray(bodyIndex=self.soccerbot.body, controlMode=pb.POSITION_CONTROL,
                                             jointIndices=list(range(0, 18, 1)),
                                             targetPositions=self.soccerbot.configuration)
                self.soccerbot.current_step_time = self.soccerbot.current_step_time + self.soccerbot.robot_path.step_size

            pb.stepSimulation()
            # self.soccerbot.get_imu()
            self.soccerbot.publishAngles()

            t = t + SoccerbotController.PYBULLET_STEP
            r.sleep()

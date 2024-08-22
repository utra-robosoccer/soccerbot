import rospy
from geometry_msgs.msg import PoseStamped
from soccer_pycontrol.model.model_ros.bez_ros import BezROS
from soccer_pycontrol.walk_engine.walk_engine_placo import WalkEnginePlaco
from soccer_pycontrol.walk_engine.walk_engine_ros.stabilize_ros import StabilizeROS
from soccer_pycontrol.walk_engine.walk_placo import WalkPlaco


class WalkEnginePlacoRos(WalkEnginePlaco):
    def __init__(self, bez: BezROS):
        self.bez = bez

        self.walk_placo = WalkPlaco("bez1", rospy.get_time)
        self.rate = rospy.Rate(1 / self.walk_placo.DT)
        self.pid = StabilizeROS()
        self.position_subscriber = rospy.Subscriber("goal", PoseStamped, self.goal_callback)
        self.goal = PoseStamped()

    def goal_callback(self, pose: PoseStamped) -> None:
        """
        Callback function for when a new goal arrives. It creates the path in the callback function and dynamically
        updates the current path if it exists (currently not working). Note the path computation occurs here
        instead of run because the computation will disturb the main thread. Main thread should still be running

        :param pose: The pose sent by the strategy for the robot to go to
        """

        self.goal = pose
        self.walk_placo.configure_planner(d_x=0.03)

    def wait(self, steps: int):
        for i in range(steps):
            rospy.sleep(self.walk_placo.DT)

    def walk(self, d_x: float = 0.0, d_y: float = 0.0, d_theta: float = 0.0, nb_steps: int = 10, t_goal: float = 10):
        self.walk_placo.setup_walk(d_x, d_y, d_theta, nb_steps)
        self.pid.reset_imus()
        t = 0
        while t < t_goal:  # TODO needs end condition
            if self.bez.sensors.imu_ready:
                [_, pitch, roll] = self.bez.sensors.get_euler_angles()

                self.walk_placo.walk_loop(t)

                joints = [0] * self.bez.motor_control.numb_of_motors
                for joint in self.bez.motor_control.motor_names:
                    if joint in ["head_base_frame", "trunk_frame", "camera_frame", "left_foot_frame", "right_foot_frame", "/torso_imu"]:
                        continue
                    joints[self.bez.motor_control.motor_names.index(joint)] = self.walk_placo.robot.get_joint(joint)

                self.bez.motor_control.configuration = joints
                self.stabilize_walk(pitch, roll)

                self.bez.motor_control.set_motor()

                self.rate.sleep()

                t = self.walk_placo.step(t)

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from soccer_pycontrol.walk_engine.foot_step_planner import FootStepPlanner


class FootStepPlannerROS(FootStepPlanner):
    def __init__(
        self,
        walking_torso_height: float = 0.315,
        foot_center_to_floor: float = 0.0221,
    ):

        torso_offset_pitch = rospy.get_param("torso_offset_pitch", 0.0)
        torso_offset_x = rospy.get_param("torso_offset_x", 0)
        # TODO create path ros
        super(FootStepPlannerROS, self).__init__(
            torso_offset_pitch=torso_offset_pitch,
            torso_offset_x=torso_offset_x,
            walking_torso_height=walking_torso_height,
            foot_center_to_floor=foot_center_to_floor,
        )
        self.path_publisher = rospy.Publisher("path", Path, queue_size=1, latch=True)
        self.path_odom_publisher = rospy.Publisher("path_odom", Path, queue_size=1, latch=True)

    def get_next_step(self, t):
        torso_to_right_foot, torso_to_left_foot = super(FootStepPlannerROS, self).get_next_step(t)

        # Get odom from odom_path
        self.odom_pose = (
            self.odom_pose_start_path
            @ self.robot_path.start_transformed_inv
            @ self.robot_path.torsoPosition(t, invert_calibration=True)
            @ self.torso_offset
        )
        return torso_to_right_foot, torso_to_left_foot

    def publishPath(self, robot_path=None):
        """
        Publishes the robot path to rviz for debugging and visualization

        :param robot_path: The path to publish, leave empty to publish the robot's current path
        """

        if robot_path is None:
            robot_path = self.robot_path

        def createPath(robot_path, invert_calibration=False) -> Path:
            p = Path()
            p.header.frame_id = "world"
            p.header.stamp = rospy.Time.now()
            for i in range(0, robot_path.torsoStepCount(), 1):
                step = robot_path.getTorsoStepPose(i)
                # if invert_calibration:
                #     step = adjust_navigation_transform(robot_path.start_transform, step)

                position = step.position
                orientation = step.quaternion
                pose = PoseStamped()
                pose.header.seq = i
                pose.header.frame_id = "world"
                pose.pose.position.x = position[0]
                pose.pose.position.y = position[1]
                pose.pose.position.z = position[2]

                pose.pose.orientation.x = orientation[0]
                pose.pose.orientation.y = orientation[1]
                pose.pose.orientation.z = orientation[2]
                pose.pose.orientation.w = orientation[3]
                p.poses.append(pose)
            return p

        self.path_publisher.publish(createPath(robot_path))
        self.path_odom_publisher.publish(createPath(robot_path, invert_calibration=True))

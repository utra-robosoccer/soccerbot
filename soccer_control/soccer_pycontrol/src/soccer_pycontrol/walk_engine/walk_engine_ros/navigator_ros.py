import rospy
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import ColorRGBA
# from pybullet_examples.getClosestPoints import colorRGB
from visualization_msgs.msg import Marker, MarkerArray
from soccer_pycontrol.model.model_ros.bez_ros import BezROS
from soccer_pycontrol.walk_engine.foot_step_planner import FootStepPlanner
from soccer_pycontrol.walk_engine.navigator import Navigator
from soccer_pycontrol.walk_engine.stabilize import Stabilize

from soccer_common import PID


class NavigatorRos(Navigator):
    def __init__(self, bez: BezROS, imu_feedback_enabled: bool = False):
        self.imu_feedback_enabled = imu_feedback_enabled
        self.bez = bez

        self.foot_step_planner = FootStepPlanner(self.bez.robot_model, self.bez.parameters, rospy.get_time)
        # TODO publish local odomtry from foot step planner
        self.rate = rospy.Rate(1 / self.foot_step_planner.DT)
        self.func_step = self.rate.sleep

        self.walk_pid = Stabilize(self.bez.parameters)
        self.max_vel = 0.09
        self.nav_x_pid = PID(
            Kp=0.5,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-self.max_vel, self.max_vel),
        )
        self.nav_y_pid = PID(  # TODO properly tune later
            Kp=0.5,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-0.05, 0.05),
        )  # TODO could also mod if balance is decreasing
        self.nav_yaw_pid = PID(
            Kp=0.2,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-0.3, 0.3),
        )

        self.error_tol = 0.05  # in m TODO add as a param and in the ros version
        self.position_subscriber = rospy.Subscriber(self.bez.ns + "goal", PoseStamped, self.goal_callback)
        self.polygon_publisher = rospy.Publisher(self.bez.ns + "polygons", MarkerArray, queue_size=1) # set queue_size to 1, look at marker
        self.goal = PoseStamped()

    def goal_callback(self, pose: PoseStamped) -> None:
        """
        Callback function for when a new goal arrives. It creates the path in the callback function and dynamically
        updates the current path if it exists (currently not working). Note the path computation occurs here
        instead of run because the computation will disturb the main thread. Main thread should still be running

        :param pose: The pose sent by the strategy for the robot to go to
        """

        self.goal = pose
        self.foot_step_planner.configure_planner(d_x=0.03)

    def wait(self, steps: int):
        for i in range(steps):
            rospy.sleep(self.foot_step_planner.DT)

    def walk_loop(self, t):
        t= super(NavigatorRos, self).walk_loop(t)
        polygons = MarkerArray()
        for i, polygon in enumerate(self.foot_step_planner.footstep_polygons):
            marker = Marker()
            marker.header.frame_id = "map" # or map
            marker.header.stamp = rospy.Time.now()
            marker.type = marker.LINE_STRIP
            marker.id = i
            marker.action = marker.ADD
            marker.scale.x = 0.01
            # marker.color.b = 1.0
            marker.color.a = 1.0
            r = ColorRGBA(1, 0, 0, 1)
            g = ColorRGBA(0, 1, 0, 1)
            b = ColorRGBA(0, 0, 1, 1)
            p1 = Point(polygon[0][0], polygon[0][1], polygon[0][2])
            p2 = Point(polygon[1][0], polygon[1][1], polygon[1][2])
            p3 = Point(polygon[2][0], polygon[2][1], polygon[2][2])
            p4 = Point(polygon[3][0], polygon[3][1], polygon[3][2])
            p5 = Point(polygon[4][0], polygon[4][1], polygon[4][2])

            marker.points.extend([p1,p2,p2,p3,p3,p4,p4,p5,p5,p1])
            marker.colors.extend([r,r,g,g,b,b,g,g,g,g])

            polygons.markers.append(marker)

        self.polygon_publisher.publish(polygons)
        return t


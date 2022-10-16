import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Transform as GeometryMsgsTransform
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


class Transformation(np.ndarray):
    """
    3D transformation or pose of an object represented as a 4x4 matrix, but can take in many formats
    """

    def __new__(
        cls,
        position=(0.0, 0.0, 0.0),
        quaternion=(0.0, 0.0, 0.0, 1.0),
        rotation_matrix: np.array = None,
        matrix: np.array = None,
        euler: np.array = None,
        pos_theta: np.array = None,
        pose: Pose = None,
        pose_stamped: PoseStamped = None,
        geometry_msgs_transform: GeometryMsgsTransform = None,
        dh: np.array = None,
        timestamp: rospy.Time = None,
        *args,
        **kwargs
    ):
        """
        Constructor for the Transformation object

        :param position: Position represented as (x, y, z) in meters
        :param quaternion: Quaternions represented in (x, y, z, w)
        :param rotation_matrix: 3x3 rotation matrix of the transform, can be used in conjunction with position
        :param matrix: 4x4 transformation matrix, includes both position and rotation
        :param euler: Orientation represented in euler format (roll, pitch, yaw)
        :param pos_theta: 2D position and theta format (x, y, theta/yaw)
        :param pose: Takes a geometry_msg.Pose object and converts it
        :type pose: :class:`Transformation`
        :param dh: (d, theta, r, alpha)
        See `DH <https://en.wikipedia.org/wiki/Field_of_view>`_
        :param args: Additional arguments, passed down to the numpy array object
        :param kwargs: Additional keyword arguments, passed down to the numpy array object
        """
        cls = np.eye(4).view(cls)

        cls.timestamp = timestamp

        if matrix is not None:
            cls.matrix = matrix
        elif rotation_matrix is not None:
            cls[0:3, 0:3] = rotation_matrix
            cls.position = position
        elif dh is not None:
            a = dh[0]
            alpha = dh[1]
            d = dh[2]
            theta = dh[3]

            cls[0, 0] = np.cos(theta)
            cls[0, 1] = -np.sin(theta) * np.cos(alpha)
            cls[0, 2] = np.sin(theta) * np.sin(alpha)
            cls[0, 3] = a * np.cos(theta)
            cls[1, 0] = np.sin(theta)
            cls[1, 1] = np.cos(theta) * np.cos(alpha)
            cls[1, 2] = -np.cos(theta) * np.sin(alpha)
            cls[1, 3] = a * np.sin(theta)
            cls[2, 1] = np.sin(alpha)
            cls[2, 2] = np.cos(alpha)
            cls[2, 3] = d
        elif euler is not None:
            cls.orientation_euler = euler
            cls.position = position
        elif pos_theta is not None:
            cls.pos_theta = pos_theta
        elif pose is not None:
            cls.pose = pose
        elif geometry_msgs_transform is not None:
            cls.geometry_msgs_transform = geometry_msgs_transform
        elif pose_stamped is not None:
            cls.pose = pose_stamped.pose
            cls.timestamp = pose_stamped.header.stamp
        else:
            cls.position = position
            cls.quaternion = quaternion
        return cls

    @property
    def matrix(self) -> np.ndarray:
        """
        Representation of the transformation in quaternion in 4x4 transformation matrix
        """
        return np.array(self)

    @matrix.setter
    def matrix(self, matrix: np.array):
        self[0:4, 0:4] = matrix

    @property
    def position(self) -> np.ndarray:
        """
        Representation of the position of the transformation in quaternion in form [x y z]
        """
        return np.array(self[0:3, 3])

    @position.setter
    def position(self, position: [float]):
        self[0:3, 3] = position

    @property
    def norm_squared(self) -> float:
        position = self.position
        return position[0] ** 2 + position[1] ** 2 + position[2] ** 2

    @property
    def quaternion(self) -> np.ndarray:
        """
        Representation of the rotation of the transformation in quaternion in form [x y z w]
        """
        r = R.from_matrix(self[0:3, 0:3])
        return r.as_quat()

    @quaternion.setter
    def quaternion(self, quat: [float]):
        r = R.from_quat(quat)
        self[0:3, 0:3] = np.reshape(r.as_matrix(), [3, 3])

    @property
    def orientation_euler(self, orientation="ZYX") -> np.ndarray:
        """
        Representation of the rotation of the transformation in euler coordinates [yaw, pitch, roll]
        """
        e = R.from_matrix(self[0:3, 0:3])
        return e.as_euler(orientation, degrees=False)

    @orientation_euler.setter
    def orientation_euler(self, euler_array, sequence="ZYX"):
        r = R.from_euler(seq=sequence, angles=euler_array, degrees=False)
        self.quaternion = r.as_quat()

    @property
    def rotation_matrix(self) -> np.array:
        """
        Representation of the rotation of the transformation in 3x3 rotation matrix
        """
        return np.array(self[0:3, 0:3])

    @rotation_matrix.setter
    def rotation_matrix(self, rotation_matrix):
        self[0:3, 0:3] = rotation_matrix

    @property
    def pos_theta(self):
        """
        Representation of the transformation in the form  [x, y, yaw]
        """
        return np.array([self.position[0], self.position[1], self.orientation_euler[0]])

    @pos_theta.setter
    def pos_theta(self, pos_theta: [float]):
        self.position = (pos_theta[0], pos_theta[1], 0.0)
        self.orientation_euler = [pos_theta[2], 0, 0]

    @property
    def pose(self) -> Pose:
        """
        Representation of the transformation in the ros Pose format
        """
        position = self.position
        quaternion = self.quaternion

        p = Pose()
        p.position.x = position[0]
        p.position.y = position[1]
        p.position.z = position[2]
        p.orientation.x = quaternion[0]
        p.orientation.y = quaternion[1]
        p.orientation.z = quaternion[2]
        p.orientation.w = quaternion[3]
        return p

    @pose.setter
    def pose(self, pose: Pose):
        self.position = [pose.position.x, pose.position.y, pose.position.z]
        self.quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    @property
    def geometry_msgs_transform(self) -> GeometryMsgsTransform:
        """
        Representation of the transformation in the ros geometry_msgs Transform format
        """
        position = self.position
        quaternion = self.quaternion

        p = GeometryMsgsTransform()
        p.translation.x = position[0]
        p.translation.y = position[1]
        p.translation.z = position[2]
        p.rotation.x = quaternion[0]
        p.rotation.y = quaternion[1]
        p.rotation.z = quaternion[2]
        p.rotation.w = quaternion[3]
        return p

    @geometry_msgs_transform.setter
    def geometry_msgs_transform(self, geometry_msgs_transform: GeometryMsgsTransform):
        self.position = [geometry_msgs_transform.translation.x, geometry_msgs_transform.translation.y, geometry_msgs_transform.translation.z]
        self.quaternion = [
            geometry_msgs_transform.rotation.x,
            geometry_msgs_transform.rotation.y,
            geometry_msgs_transform.rotation.z,
            geometry_msgs_transform.rotation.w,
        ]

    @property
    def pose_stamped(self) -> PoseStamped:
        """
        The transformation represented in the PoseStamped format
        """
        t = PoseStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.pose = self.pose
        return t

    @staticmethod
    def distance(t1, t2) -> float:
        """
        Returns the distance between two transformations
        :param t1: transformation 1
        :type t1: Transformation
        :param t2: transformation 2
        :type t1: Transformation
        :return: The distance in meters
        """
        return np.linalg.norm(t1[0:3, 3] - t2[0:3, 3])

    @staticmethod
    def get_euler_from_quaternion(quaternion, seq="ZYX"):
        """
        Get the quaternion representation of euler angle rotations

        :param euler_array: array of 3 angles for rotation
        :param sequence: order and type of rotation, see intrinsic vs extrinsic for capital and small case letter
        :return: quaternion in the form of [x y z w]
        """
        r = R.from_quat(quaternion)
        return r.as_euler(seq=seq)

    @staticmethod
    def get_quaternion_from_axis_angle(vector, angle):
        """
        Gives the quaternion representation of axis-angle rotation

        :param vector: vector around which the rotation takes place
        :param angle: angle by which is rotated around the vector
        :return: quaternion in the form of [x y z w]
        """
        r = R.from_rotvec(np.array(vector).reshape((1, 3)) * (angle / np.linalg.norm(vector)))
        return r.as_quat()

    @staticmethod
    def get_axis_angle_from_quaternion(quaternion):
        """
        Gives the axis-angle representation of rotation from a quaternion

        :param quaternion: quaternion in the form of [x y z w]
        :return: angle and vector
        """
        r = R.from_quat(quaternion)
        angle = np.linalg.norm(r.as_rotvec())
        if angle:
            vector = r.as_rotvec() / angle
        else:
            vector = [0, 0, 1]
        return angle, vector

    @staticmethod
    def transformation_weighted_average(t_start, t_end, ratio):
        """
        Interpolates between two transforms. Inclination is based on a ratio.
        Ratio = 0, t_start shall be returned
        Ratio = 1, t_end shall be returned

        :param t_start: start H-transform
        :param t_end: end H-transform
        :param ratio: a number between 0 and 1
        :return: weighted average H-transform
        """
        average = Transformation()
        delta_position = t_end.position - t_start.position
        average.position = t_start.position + (delta_position * ratio)
        rots = R.from_quat([t_start.quaternion, t_end.quaternion])
        s = Slerp([0, 1], rots)
        r_average = s([ratio])[0]

        average.quaternion = r_average.as_quat()

        return average

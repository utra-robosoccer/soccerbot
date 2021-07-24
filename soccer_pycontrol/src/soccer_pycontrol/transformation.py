import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


class Transformation(np.ndarray):

    def __new__(cls, position=(0., 0., 0.), quaternion=(0., 0., 0., 1.), *args, **kwargs):
        """
        Constructor for the H-transform object, inherits from numpy array
        :param position: translation component of the transform, defaults to zero
        :param quaternion: rotational component of the transform in quaternion of form [x y z w], defaults to no rotation
        """
        cls = np.eye(4).view(cls)
        cls.set_position(position)
        cls.set_orientation(quaternion)
        return cls

    def get_transform(self):
        """
        Returns the np array for this
        :return: a vector 3x1
        """
        return np.array(self)

    def get_position(self):
        """
        Gives the translational component of H-transform
        :return: a vector 3x1
        """
        return np.array(self[0:3, 3])

    def set_position(self, position):
        """
        sets the translational component of H-transform
        :param position: a vector 3x1
        :return: None
        """
        self[0:3, 3] = position

    def get_orientation(self):
        """
        Gives the rotation of the H-transform in quaternion form
        :return: quaternion of form [x y z w]
        """
        r = R.from_matrix(self[0:3, 0:3])
        return r.as_quat()

    def get_orientation_euler(self):
        """
        Gives the rotation of the H-transform in euler axis form
        :return: angles of form [r p y]
        """
        return Transformation.get_euler_from_rotation_matrix(self[0:3, 0:3])

    def set_orientation(self, quat):
        """
        Sets the rotational component of the H-transform
        :param quat: rotation in the quaternion form of [x y z w]
        :return: None
        """
        r = R.from_quat(quat)
        self[0:3, 0:3] = np.reshape(r.as_matrix(), [3, 3])

    @staticmethod
    def get_transform_from_dh(a, alpha, d, theta):
        t = Transformation()

        t[0, 0] = np.cos(theta)
        t[0, 1] = - np.sin(theta) * np.cos(alpha)
        t[0, 2] = np.sin(theta) * np.sin(alpha)
        t[0, 3] = a * np.cos(theta)
        t[1, 0] = np.sin(theta)
        t[1, 1] = np.cos(theta) * np.cos(alpha)
        t[1, 2] = - np.cos(theta) * np.sin(alpha)
        t[1, 3] = a * np.sin(theta)
        t[2, 1] = np.sin(alpha)
        t[2, 2] = np.cos(alpha)
        t[2, 3] = d

        return t

    @staticmethod
    def get_transform_from_pose_stamped(x, y, theta):
        t = Transformation.get_transform_from_euler([0, 0, theta])
        t.set_position([x, y, 0])
        return t

    @staticmethod
    def get_transform_from_euler(euler_array, sequence='ZYX'):
        """
        Gives 4x4 H-transform object from the euler angle represtation with translation component defaulting to zero
        :param euler_array: the array of 3 angles
        :param sequence: axes of rotation order in a string of three letters, see intrinsic vs extrinsic for capital and small case letters
        :return: H-transform
        """
        r = R.from_euler(seq=sequence, angles=euler_array, degrees=False)
        t = Transformation(quaternion=r.as_quat())
        return t

    @staticmethod
    def get_euler_from_rotation_matrix(r, orientation='ZYX'):
        """
        Gives euler angles rotation form of rotation matrix
        :param r: 3x3 rotation matrix
        :param orientation: axes of rotation order in a string of three letters, see intrinsic vs extrinsic for capital and small case letters
        :return: an array of three angles in radians
        """
        e = R.from_matrix(r)
        return e.as_euler(orientation, degrees=False)

    @staticmethod
    def get_quaternion_from_euler(euler_array, sequence='ZYX'):
        """
        Get the quaternion representation of euler angle rotations
        :param euler_array: array of 3 angles for rotation
        :param sequence: order and type of rotation, see intrinsic vs extrinsic for capital and small case letter
        :return: quaternion in the form of [x y z w]
        """
        r = R.from_euler(seq=sequence, angles=euler_array, degrees=False)
        return r.as_quat()

    @staticmethod
    def get_quaternion_from_axis_angle(vector, angle):
        """
        Gives the quaternion representation of axis-angle rotation
        :param vector: vector around which the rotation takes place
        :param angle: angle by which is rotated around the vector
        :return: quaternion in the form of [x y z w]
        """
        r = R.from_rotvec(np.array(vector).reshape((1,3)) * (angle / np.linalg.norm(vector)))
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
        np.seterr(invalid = "raise")
        try:
            vector = r.as_rotvec() / np.linalg.norm(r.as_rotvec())
        except:
            vector = [0, 0, 1]
        return angle, vector

    @staticmethod
    def get_distance(t1, t2):
        """
        Gives the translational distance between 2 H-transforms
        :param t1: first H-transform
        :param t2: second H-transform
        :return:
        """
        return np.linalg.norm(t1[0:3, 3] - t2[0: 3, 3])

    @staticmethod
    def get_rotation_matrix_from_transformation(transformation):
        """
        Returns a 3x3 rotation matrix from the 4x4 H-transform
        :param transformation: H-transform
        :return: rotation matrix
        """
        r = R.from_quat(transformation.get_orientation())
        return r.as_matrix()

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
        delta_position = t_end.get_position() - t_start.get_position()
        average.set_position(t_start.get_position() + (delta_position * ratio))
        rots = R.from_quat([t_start.get_orientation(), t_end.get_orientation()])
        s = Slerp([0, 1], rots)
        r_average = s([ratio])[0]

        average.set_orientation(r_average.as_quat())

        return average

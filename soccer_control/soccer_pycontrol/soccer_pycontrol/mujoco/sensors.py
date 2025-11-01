import meshcat.transformations as tf
import mujoco
import numpy as np

from soccer_common import Transformation


class Sensors:
    """
    Interfaces with MuJoCo to extract sensor data.
    """

    def __init__(self, world):
        # TODO does this need to be a class?
        self.world = world

        # TODO should get based on name
        self.imu_ready = False
        # self.get_imu()  # to init

    def get_pose(self) -> Transformation:
        """
        Get the 3D pose of the robot in the world frame.

        :return: The 3D pose of the robot
        """
        T = self.world.get_T_world_site("torso")
        return Transformation(matrix=T)

    def get_ball_local_frame(self):
        robot_T = self.get_pose()
        ball_T = self.get_ball_world_frame()
        trans = (np.array(ball_T.position) - np.array(robot_T.position)).tolist()
        return Transformation(position=trans, quaternion=ball_T.quaternion)

    def get_ball_world_frame(self):
        T = self.world.get_T_world_site("ball")
        return Transformation(matrix=T)

    # def get_camera_image(self):
    #     """
    #     Captures the image from the camera mounted on the robot
    #     """
    #     # Add more offsets later
    #     camera_position = self.get_pose(link=2).position
    #
    #     # print(f"Pos: {camera_position} Orient: {self.get_pose(link=2).orientation_euler}")
    #     view_matrix = pb.computeViewMatrixFromYawPitchRoll(
    #         camera_position,
    #         0.000367,
    #         pb.getJointState(self.body, 0)[0] * (180 / np.pi) - 90,  # self.get_pose(link=2).orientation_euler[0]*(180/np.pi) + 90,
    #         -pb.getJointState(self.body, 1)[0] * (180 / np.pi),  # self.get_pose(link=2).orientation_euler[1]*(180/np.pi)+90,
    #         0,  # self.get_pose(link=2).orientation_euler[2]*(180/np.pi) ,
    #         2,
    #     )
    #     width, height = 640, 480
    #     fov = 78
    #     aspect = width / height
    #     near = 0.2
    #     far = 100
    #
    #     projection_matrix = pb.computeProjectionMatrixFOV(fov, aspect, near, far)
    #
    #     images = pb.getCameraImage(width, height, view_matrix, projection_matrix, shadow=False, renderer=pb.ER_BULLET_HARDWARE_OPENGL)
    #
    #     # NOTE: the ordering of height and width change based on the conversion
    #     img = np.reshape(images[2], (height, width, 4))
    #     img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    #     # rgb_opengl = np.reshape(images[2], (height, width, 4))[:, :, :3] * 1. / 255.
    #     # depth_buffer_opengl = np.reshape(images[3], [width, height])
    #     # depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)
    #     # seg_opengl = np.reshape(images[4], [width, height]) * 1. / 255.
    #
    #     return img

    def get_height(self):
        return

    def centroidal_force(self):
        return np.linalg.norm(self.world.data.qfrc_constraint[3:])

    def get_gyro(self) -> np.ndarray:
        """
        Gets the gyroscope data.

        Returns:
            np.ndarray: gyroscope data
        """
        return self.world.data.sensor("gyro").data

    def get_accel(self) -> np.ndarray:
        """
        Gets the gyroscope data.

        Returns:
            np.ndarray: gyroscope data
        """
        return self.world.data.sensor("accelerometer").data

    def get_foot_pressure_sensors(self) -> dict:
        left_pressures = [
            -self.world.data.sensor("left_foot_cleat_front_right").data[2],
            -self.world.data.sensor("left_foot_cleat_front_left").data[2],
            -self.world.data.sensor("left_foot_cleat_back_right").data[2],
            -self.world.data.sensor("left_foot_cleat_back_left").data[2],
        ]
        right_pressures = [
            -self.world.data.sensor("right_foot_cleat_front_right").data[2],
            -self.world.data.sensor("right_foot_cleat_front_left").data[2],
            -self.world.data.sensor("right_foot_cleat_back_right").data[2],
            -self.world.data.sensor("right_foot_cleat_back_left").data[2],
        ]

        return {"left": left_pressures, "right": right_pressures}

    def get_head_height(self):  # TODO add a transfomration from imu link
        left_foot = self.world.get_T_world_site("left_foot")[0:3][:, 3]
        right_foot = self.world.get_T_world_site("right_foot")[0:3][:, 3]
        foot = (left_foot + right_foot) / 2
        head_height = self.world.get_T_world_site("camera")[2][3] - foot[2]
        if self.world.get_T_world_site("camera")[2][3] < foot[2]:
            head_height = -10
        return head_height

    def get_rpy(self, site: str = "trunk") -> np.ndarray:
        R = self.world.data.site(site).xmat
        # pitch = np.arctan2(-R[6], np.sqrt(R[0] ** 2 + R[3] ** 2)) #
        pitch = -np.arctan2(R[6], R[8])
        roll = np.arctan2(R[7], R[8])  # atan2(R[2,1], R[2,2])
        yaw = np.arctan2(R[3], R[0])
        # fused, tilt = self.fused_tilt_from_xmat(R)
        # psi, theta, phi, h = fused

        # return np.array([phi, psi, theta])
        return np.array([roll, pitch, yaw])

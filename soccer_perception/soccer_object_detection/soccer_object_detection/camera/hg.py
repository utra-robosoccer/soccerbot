import numpy as np
from geometry_msgs.msg import TransformStamped

from soccer_common import Transformation
from soccer_object_detection.camera.camera_calculations import CameraCalculations

if __name__ == '__main__':
    cam = CameraCalculations()
    # cam.pose.position = [-0.029215,   0.0008118,     0.53978]
    cam.pose.position = [0, 0, 0.53978]
    # cam.pose.orientation_euler = [-1.57, 0, -1.57]
    cam.pose.orientation_euler = [0, 0.22717, 0]
    # cam.pose.rotation_matrix = cam.pose.rotation_matrix.T
    # cam.pose.orientation_euler = [3.14153443 ,1.34442265, 3.14134969]
    # ang = -1.57 - 2*0.22717
    # rot = Transformation(euler=[0, 0.22717, 0])
    # cam.pose.rotation_matrix = rot.rotation_matrix@ cam.pose.rotation_matrix
    # rot = Transformation(euler=[0, 0, -1.57])
    # cam.pose.rotation_matrix = rot.rotation_matrix @ cam.pose.rotation_matrix
    print(cam.pose.position, " ", cam.pose.orientation_euler)
    #
    print(-1* cam.pose.rotation_matrix.T @  cam.pose.position)
    print( np.linalg.inv(cam.pose))
    # cam.pose = np.linalg.inv(cam.pose)
    # print(cam.pose.rotation_matrix)
    # cam.pose.rotation_matrix = cam.pose.rotation_matrix.T
    # print(cam.get_intrinsic_matrix())
    yy = cam.map_point(407,405)
    print(f"{yy[0]}, {yy[1]}, {yy[2]}" )
    pass

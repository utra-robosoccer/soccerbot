import numpy as np

from soccer_object_detection.camera.camera_calculations import CameraCalculations

if __name__ == '__main__':
    cam = CameraCalculations()
    cam.pose.position = [0,0,-1]
    cam.pose.orientation_euler = [0,1,0]
    cam.pose.rotation_matrix = cam.pose.rotation_matrix.T
    print(cam.pose.quaternion)
    print(-1* cam.pose.rotation_matrix.T @  cam.pose.position)
    print( np.linalg.inv(cam.pose))
    cam.pose = np.linalg.inv(cam.pose)
    # print(cam.pose.rotation_matrix)
    # cam.pose.rotation_matrix = cam.pose.rotation_matrix.T

    yy = cam.map_point(0, 0)
    print(yy)
    pass
import time
import unittest
from os.path import expanduser

import cv2

from soccer_object_detection.object_detect_node import ObjectDetectionNode
from soccer_pycontrol.model.bez import Bez  # Import the Bez class
from soccer_pycontrol.model.sim_world import SimWorld


class TestSensors(unittest.TestCase):

    def test_camera_image_capture(self):
        sim = SimWorld()
        bez = Bez(sim)
        start = time.time()

        while sim.t < 200:
            sim.render(True)

            sim.step()

            elapsed = time.time() - start
            frames = sim.frame
            cv2.namedWindow("ball_cam", cv2.WINDOW_NORMAL)
            cv2.imshow("ball_cam", bez.sensors.get_camera_image())
            if cv2.waitKey(1) & 0xFF == 27:  # ESC
                break
            [_, pitch, roll] = bez.sensors.get_imu()
            print(bez.fallen(pitch, roll))  # TODO add assert

            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()
    def test_ball_localization(self): # TODO could extend to robots and goals
        src_path = expanduser("~") + "/ros2_ws/src/soccerbot/soccer_perception/"
        model_path = src_path + "soccer_object_detection/models/half_5.pt"

        detect = ObjectDetectionNode(model_path)

        sim = SimWorld()
        bez = Bez(sim)
        # bez.motor_control.set_single_motor("head_pitch", 0.1)
        # bez.motor_control.set_motor()
        start = time.time()
        cam_dt = 1.0/30.0
        n_substeps = int(round(cam_dt / sim.dt))
        while sim.t < 300:
            if sim.frame % n_substeps == 0:
                img = bez.sensors.get_camera_image()
                img = cv2.resize(img, dsize=(640, 480))
                start = time.time()
                dimg, bbs_msg = detect.get_model_output(img)

                for box in bbs_msg.bounding_boxes:
                    if box.class_id == "0":
                        detect.camera.pose.position = [0, 0, bez.sensors.get_cam_pose().position[2]]
                        detect.camera.pose.orientation_euler = bez.sensors.get_cam_pose().orientation_euler
                        boundingBoxes = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
                        pos = [box.xbase, box.ybase]
                        floor_coordinate_robot = detect.camera.find_floor_coordinate(pos)
                        xx = detect.camera.uv_to_roadXYZ_camframe(box.xbase,box.ybase)
                        yy = detect.camera.map_point(box.xbase,box.ybase)
                        string = (
                                 # f"z: {bez.sensors.get_cam_pose().position[2]}  eul: {bez.sensors.get_cam_pose().orientation_euler}" +
                                  f" floor pos: {detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes).position}" +
                                  f" floor pos2: {floor_coordinate_robot} " +
                                  f" floor pos3: {xx} " +
                                  f" floor pos4: {yy} " #+
                                  # f" pos2: {bez.sensors.get_pose().rotation_matrix @ bez.sensors.get_ball_local_frame().position + bez.sensors.get_pose().position} ball: {bez.sensors.get_ball_world_frame().position}"
                                  )

                        print(
                            string,#end='\r',
                            flush=True,
                        )


                # if "DISPLAY" in os.environ:
                cv2.imshow("CVT Color2", dimg)
                cv2.waitKey(1)

            elapsed = time.time() - start
            frames = sim.frame
            sim.render(True)
            sim.step()

            # print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()

if __name__ == "__main__":
    unittest.main()
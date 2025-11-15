import time
import unittest
from os.path import expanduser
from random import uniform

import cv2
import numpy as np

from soccer_object_detection.object_detect_node import ObjectDetectionNode
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.model.sim_world import SimWorld

from soccer_common import Transformation
from soccer_pycontrol.walk_engine.navigator import Navigator
from soccer_trajectories.trajectory_manager_sim import TrajectoryManagerSim

REAL_TIME = True


class TestMuJoCo(unittest.TestCase):
    def test_imu(self):
        sim = SimWorld(keyframe="fallen_side_right")
        bez = Bez(sim)
        start = time.time()

        while sim.t < 3000:
            sim.render(True)

            sim.step()

            elapsed = time.time() - start
            frames = sim.frame
            [_, pitch, roll] = bez.sensors.get_imu()
            print(bez.fallen(pitch, roll))  # TODO add assert

            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()

    def test_cam(self):
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

    def test_ball_pose(self):
        # sim = SimWorld()
        sim = SimWorld(pose=Transformation(position=[1, 1,0.3975], euler=[0, 0, 0]))
        bez = Bez(sim)
        sim.set_T_world_ball(Transformation(position=[2, -1, 0.14], euler=[0, 0, 0]))
        sim.step()
        start = time.time()

        while sim.t < 20:
            sim.render(True)

            sim.step()

            elapsed = time.time() - start
            frames = sim.frame
            print("pose: ", bez.sensors.get_pose().position)
            print("local: ", bez.sensors.get_ball_local_frame().position)  # TODO add assert
            print("Global: ", bez.sensors.get_ball_world_frame().position)
            print("2Global: ", bez.sensors.get_head_height())
            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()

    def test_bez_motor_range_single(self):
        sim = SimWorld(keyframe="float")
        bez = Bez(sim)

        start = time.time()
        angles = np.linspace(-np.pi, np.pi)

        sim.render(True)

        sim.step()
        name = "head_pitch"
        # name = "left_hip_roll"

        self.motor_range(angles, name, sim, bez, sim.T)

        sim.wait(1000)
        elapsed = time.time() - start
        frames = sim.frame

        print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")
        sim.close_viewer()

    def test_bez_motor_range(self):
        sim = SimWorld(keyframe="float")
        bez = Bez(sim)

        start = time.time()
        angles = np.linspace(-np.pi, np.pi)

        sim.render(True)

        sim.step()

        for name in bez.motor_control.dof_names:
            bez.motor_control.clear_motor_target()
            bez.motor_control.reset_motors(np.zeros_like(bez.motor_control.configuration))
            sim.render(True)
            sim.step()
            print(name)
            self.motor_range(angles, name, sim, bez, sim.T)

        sim.wait(1000)
        elapsed = time.time() - start
        frames = sim.frame

        print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()

    def motor_range(self, angles: np.ndarray, name: str, sim:SimWorld, bez: Bez, T):
        angle_idx = 0
        while angles[angle_idx] != angles[-1]:
            while bez.motor_control.get_q(name) != angles[angle_idx]:
                bez.motor_control.set_single_motor(name, angles[angle_idx])

                # bez.motor_control.set_single_motor(name.replace("left", "right"), angles[angle_idx])
                bez.motor_control.set_motor()
                print(angles[angle_idx])
                # print(
                #     f" current Target angle: {angles[angle_idx]} {angle_idx} current joint angle: {bez.motor_control.get_q(name)}  current joint angle con: {bez.motor_control.get_control(name)}"
                # )
                sim.render(True)
                sim.step()

                if angles[angle_idx] < bez.motor_control.get_range(name)[0] or angles[angle_idx] > \
                        bez.motor_control.get_range(name)[1] or abs(bez.motor_control.get_qdot(name)) < 0.01:
                    break

            angle_idx += 1

            for i in range(10):
                sim.set_T_world_site("torso", T.matrix) # todo maybe could make this more of a built in fucntion
                sim.render(True)
                sim.step()

    # def test_foot_sensor(self): # TODO add to mcf model
    #     sim = SimWorld()
    #     bez = Bez(sim)
    #     sim.step()
    #     T = np.eye(4)
    #     # T = Transformation(position=[0, 0, 0.070], euler=[0, -1.57, 0])
    #     # T = Transformation(position=[0, 0, 0.070],
    #     #                    euler=[0, 1.57, 0])  # TODO maybe should be functions in world to streamline
    #     sim.set_T_world_site("left_foot", T)
    #
    #     sim.step()
    #     start = time.time()
    #
    #     while sim.t < 10:
    #         sim.render(True)
    #
    #         sim.step()
    #
    #         elapsed = time.time() - start
    #         frames = sim.frame
    #         print(bez.sensors.get_foot_pressure_sensors())
    #
    #          # TODO add assert
    #
    #         print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")


    def test_walk(self):
        sim = SimWorld()
        bez = Bez(sim)
        walk = Navigator(sim, bez, imu_feedback_enabled=True)
        walk.ready()
        walk.world.step()
        sim.wait(200)
        # walk.wait(100)
        target_goal = [0, 0.1, 0, 10, 500]
        # target_goal = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        start = time.time()
        print("STARTING WALK")
        s = time.time()
        while sim.t < 20:

            if sim.frame % int(walk.foot_step_planner.DT/sim.dt) == 0: # TODO investigate interp
                walk.walk(target_goal, display_metrics=False)
                print("here: " + str(1/(time.time() - s)))
                s = time.time()

            sim.render(True)
            sim.step()
            elapsed = time.time() - start
            frames = sim.frame
            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()


    def test_walk_rand(self): # TODO needs tuning
        sim = SimWorld()
        bez = Bez(sim)
        walk = Navigator(sim, bez, imu_feedback_enabled=True)
        walk.ready()
        walk.world.step()
        sim.wait(200)
        # walk.wait(100)
        target_goal = [0.1, 0, 0.0, 10, 500]
        target_goal = Transformation(position=[0.07, -0.00, 0], euler=[0, 0, 0])
        start = time.time()
        print("STARTING WALK")
        while sim.t < 100:
            sim.render(True)

            sim.step()

            elapsed = time.time() - start
            frames = sim.frame
            # print(bez.sensors.get_pose().position)
            if not walk.walker.enable_walking:
                print("WALK ENABLED")
                x = uniform(-1, 1) # TODO own unit test for yaw
                y = uniform(-1, 1)
                theta = uniform(-3.14, 3.14)
                print("here: ", x, y, theta)
                target_goal = Transformation(position=[x, y, 0], euler=[theta, 0, 0])
                walk.walker.reset_walk()
            if sim.frame % int(walk.foot_step_planner.DT/sim.dt) == 0: # TODO investigate interp
                    walk.walk(target_goal, display_metrics=False)

            # walk.walk(target_goal, display_metrics=False)
            # print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()

    def test_ready(self):
        sim = SimWorld()
        bez = Bez(sim)
        walk = Navigator(sim, bez, imu_feedback_enabled=False)
        walk.ready()
        walk.world.step()
        sim.wait(200)

        start = time.time()
        print("STARTING WALK")
        while sim.t < 3:
            sim.render(True)

            sim.step()
            # cv2.namedWindow("ball_cam", cv2.WINDOW_NORMAL)
            # cv2.imshow("ball_cam", bez.sensors.get_camera_image())
            # if cv2.waitKey(1) & 0xFF == 27:  # ESC
            #     break
            elapsed = time.time() - start
            frames = sim.frame
            # print(bez.fallen(pitch))  # TODO add assert

            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()

    def test_ball_loc(self):
        src_path = expanduser("~") + "/ros2_ws/src/soccerbot/soccer_perception/"
        model_path = src_path + "soccer_object_detection/models/yolov8s_detect_best.pt"
        model_path = src_path + "soccer_object_detection/models/half_5.pt"

        detect = ObjectDetectionNode(model_path)

        sim = SimWorld()
        bez = Bez(sim)
        walk = Navigator(sim, bez, imu_feedback_enabled=False)
        walk.ready()
        walk.world.step()

        start = time.time()
        print("STARTING WALK")
        while sim.t < 300:
            if sim.frame % 20 == 0:
                img = bez.sensors.get_camera_image()
                img = cv2.resize(img, dsize=(640, 480))
                dimg, bbs_msg = detect.get_model_output(img)
                for box in bbs_msg.bounding_boxes:
                    if box.class_id == "0":
                        detect.camera.pose.position = [0, 0, bez.sensors.get_cam_pose().position[2]]
                        detect.camera.pose.orientation_euler = bez.sensors.get_cam_pose().orientation_euler
                        boundingBoxes = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
                        pos = [box.xbase, box.ybase]
                        floor_coordinate_robot = detect.camera.find_floor_coordinate(pos)
                        string = (
                                  f"z: {bez.sensors.get_cam_pose().position[2]}  eul: {bez.sensors.get_cam_pose().orientation_euler}" +
                                  f" floor pos: {detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes).position} ball: {bez.sensors.get_ball_local_frame().position}" +
                                  f" floor pos2: {floor_coordinate_robot}  ball: {bez.sensors.get_ball_local_frame().position}" +
                                  f" pos2: {bez.sensors.get_pose().rotation_matrix @ bez.sensors.get_ball_local_frame().position + bez.sensors.get_pose().position} ball: {bez.sensors.get_ball_world_frame().position}"
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
            # print(bez.fallen(pitch))  # TODO add assert

            # print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()


    def test_full(self):
        src_path = expanduser("~") + "/ros2_ws/src/soccerbot/soccer_perception/"
        model_path = src_path + "soccer_object_detection/models/yolov8s_detect_best.pt"
        model_path = src_path + "soccer_object_detection/models/half_5.pt"

        detect = ObjectDetectionNode(model_path)

        sim = SimWorld()
        bez = Bez(sim)
        tm = TrajectoryManagerSim(sim, bez, "bez2", "getupfront")
        walk = Navigator(sim, bez, imu_feedback_enabled=True)
        walk.ready()
        sim.wait(500)
        walk.world.step()

        start = time.time()
        print("STARTING WALK")
        ball_pos = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        kicked = False
        ball_pixel = [0, 0]
        while sim.t < 300:
            if sim.frame % 20 == 0:
                img = bez.sensors.get_camera_image()
                img = cv2.resize(img, dsize=(640, 480))
                dimg, bbs_msg = detect.get_model_output(img)
                for box in bbs_msg.bounding_boxes:
                    if box.class_id == "0":
                        detect.camera.pose.position = [0, 0, bez.sensors.get_cam_pose().position[2]]
                        detect.camera.pose.orientation_euler = bez.sensors.get_cam_pose().orientation_euler
                        boundingBoxes = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
                        ball_pixel = [(box.xmin + box.xmax) / 2.0, (box.ymin + box.ymax) / 2.0]
                        kicked = False
                        ball_pos = bez.sensors.get_ball_local_frame()
                        pos = [box.xbase, box.ybase]
                        floor_coordinate_robot = detect.camera.find_floor_coordinate(pos)
                        string = (
                                  f"z: {bez.sensors.get_cam_pose().position[2]}  eul: {bez.sensors.get_cam_pose().orientation_euler}" +
                                  f" floor pos: {detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes).position} ball: {bez.sensors.get_ball_local_frame().position}" +
                                  f" floor pos2: {floor_coordinate_robot}  ball: {bez.sensors.get_ball_local_frame().position}"
                                  )

                        print(
                            string,#end='\r',
                            flush=True,
                        )

                        # ball_pos = Transformation(position=floor_coordinate_robot)


                # if "DISPLAY" in os.environ:
                cv2.imshow("CVT Color2", dimg)
                cv2.waitKey(1)
            if 0 < np.linalg.norm(ball_pos.position[:2]) < 0.0 and not kicked:
                walk.ready()
                walk.wait(100)
                tm.send_trajectory("rightkick")
                kicked = True


                walk.walker.reset_walk()
            else:

                if sim.frame % int(walk.foot_step_planner.DT / sim.dt) == 0:  # TODO investigate interp
                    walk.walk(ball_pos, ball_pixel, True)
                    # print("here: " + str(1 / (time.time() - s)))
                    # s = time.time()
                    # print("ba;ll: " + str(np.linalg.norm(ball_pos.position[:2])))
                # walk.walk(ball_pos, ball_pixel, True)
                # print( "ba;ll: "+ str(np.linalg.norm(ball_pos.position[:2])) )
            elapsed = time.time() - start
            frames = sim.frame
            sim.render(True)
            sim.step()
            # print(bez.fallen(pitch))  # TODO add assert

            # print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()
    def test_bez1_start_stop(self):
        sim = SimWorld()
        bez = Bez(sim)
        walk = Navigator(sim, bez, imu_feedback_enabled=False)
        walk.ready()
        walk.world.step()
        sim.wait(200)
        # walk.wait(100)
        target_goal = [0.1, 0, 0.0, 10, 2]
        # target_goal = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        start = time.time()
        print("STARTING WALK")
        while sim.t < 10:
            sim.render(True)
            sim.step()
            elapsed = time.time() - start
            frames = sim.frame
            walk.walk(target_goal)
            if sim.t % 3 == 0:
                walk.walker.reset_walk()
            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()



    def test_bez1_auto(self):
        sim = SimWorld(keyframe="fallen_front")
        bez = Bez(sim)
        tm = TrajectoryManagerSim(sim, bez, "bez2", "getupfront")
        start = time.time()

        while sim.t < 50:
            sim.render(True)

            sim.step()

            elapsed = time.time() - start
            frames = sim.frame
            y, p, r = bez.sensors.get_imu()
            print(y, "  ", p, "  ", r)
            traj = bez.fallen(p,r)
            tm.send_trajectory(traj)


            # print(bez.fallen(pitch))  # TODO add assert

            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()

    def test_bez1_kick(self):
        sim = SimWorld()
        bez = Bez(sim)
        tm = TrajectoryManagerSim(sim, bez, "bez2", "getupfront")
        start = time.time()

        while sim.t < 50:
            sim.render(True)

            sim.step()

            elapsed = time.time() - start
            frames = sim.frame


            tm.send_trajectory("rightkick")


            # print(bez.fallen(pitch))  # TODO add assert

            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()


# TODO test stand plane - apply force

import time
import unittest
from os.path import expanduser
from random import uniform

import cv2
import mujoco
import numpy as np
from etils import epath

from soccer_object_detection.object_detect_node import ObjectDetectionNode
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.model.sim_world import SimWorld

from soccer_common import Transformation
from soccer_pycontrol.walk_engine.navigator import Navigator
from soccer_trajectories.trajectory_manager_sim import TrajectoryManagerSim
import onnxruntime as rt

from test.keyboard_gamepad import KeyboardGamepad

REAL_TIME = True
_HERE = epath.Path(__file__).parent
_ONNX_DIR = _HERE / "onnx"
class OnnxController:
  """ONNX controller for the Booster T1 humanoid."""

  def __init__(
      self,
      policy_path: str,
      # default_angles: np.ndarray,
      ctrl_dt: float,
      n_substeps: int,
      action_scale: float = 0.5,
      vel_scale_x: float = 1.0,
      vel_scale_y: float = 1.0,
      vel_scale_rot: float = 1.0,

  ):
    self._output_names = ["continuous_actions"]
    self._policy = rt.InferenceSession(
        policy_path, providers=["CPUExecutionProvider"]
    )

    self._action_scale = action_scale
    # self._default_angles = np.array([-0.00038 ,0.00058
    #             ,-0.44675 ,0.00074 ,2.50858
    #             ,-0.02901 ,0.06566 ,0.87410 ,-1.54022 ,0.67634 ,-0.07002
    #             ,-0.44962 ,-0.00074 ,2.50967
    #             ,-0.01830 ,0.04949 ,0.85711 ,-1.54784 ,0.69839 ,-0.05164])
    self._default_angles = np.array([-0.02901, 0.06566, 0.87410, -1.54022, 0.67634, -0.07002
                                        , -0.01830, 0.04949, 0.85711, -1.54784, 0.69839, -0.05164])
    self._last_action = np.zeros_like(self._default_angles, dtype=np.float32)
    self._counter = 0
    self._n_substeps = n_substeps

    self._phase = np.array([0.0, np.pi])
    self._gait_freq = 1.5
    self._phase_dt = 2 * np.pi * self._gait_freq * ctrl_dt

    # self._joystick = KeyboardGamepad(
    #     vel_scale_x=vel_scale_x,
    #     vel_scale_y=vel_scale_y,
    #     vel_scale_rot=vel_scale_rot,
    # )
    self.cmd = [0, 0 ,0]

  def get_obs(self, model, data,joint_angles, joint_velocities) -> np.ndarray:
    # linvel = data.sensor("local_linvel").data
    gyro = data.sensor("gyro").data
    imu_xmat = data.site_xmat[model.site("torso").id].reshape(3, 3)
    gravity = imu_xmat.T @ np.array([0, 0, -1])
    print(gravity)
    # print(len(data.qpos[7:]))
    # print(len(self._default_angles))

    joint_angles = joint_angles - self._default_angles # TODO will need to investigate how this scales
    # joint_velocities = data.qvel[6:18]
    phase = np.concatenate([np.cos(self._phase), np.sin(self._phase)])
    # command = [0.5, 0 ,0]#self._joystick.get_command()
    command =  self.cmd#self._joysticgfk.get_command()
    obs = np.hstack([
        # linvel,
        gyro,
        gravity,
        command,
        joint_angles,
        joint_velocities,
        self._last_action,
        phase,
    ])
    # print(len(joint_angles), len(joint_velocities))
    return obs.astype(np.float32)

  def get_control(self, model: mujoco.MjModel, data: mujoco.MjData, joint_angles, joint_velocities) :
    self._counter += 1
    if self._counter % self._n_substeps == 0:
      obs = self.get_obs(model, data, joint_angles, joint_velocities)
      onnx_input = {"obs": obs.reshape(1, -1)}
      onnx_pred = self._policy.run(self._output_names, onnx_input)[0][0]
      self._last_action = onnx_pred.copy()
      # data.ctrl[:] = onnx_pred * self._action_scale + self._default_angles
      phase_tp1 = self._phase + self._phase_dt
      self._phase = np.fmod(phase_tp1 + np.pi, 2 * np.pi) - np.pi
      return onnx_pred * self._action_scale + self._default_angles
    return None
class TestMuJoCo(unittest.TestCase):
    def test_imu(self):
        sim = SimWorld(keyframe="stand")
        bez = Bez(sim)
        start = time.time()

        while sim.t < 3000:
            sim.render(True)

            sim.step()

            elapsed = time.time() - start
            frames = sim.frame
            [_, pitch, roll] = bez.sensors.get_imu()
            print(bez.fallen(pitch, roll))  # TODO add assert
            imu_xmat = sim.data.site_xmat[sim.model.site("torso").id].reshape(3, 3)
            gravity = imu_xmat.T @ np.array([0, 0, -1])
            g2 = bez.sensors.get_pose().rotation_matrix.T @ np.array([0, 0, -1], dtype=np.float32)
            print(f" gravity: {gravity}, g2: {g2}")
            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()


    def test_on(self):
        sim = SimWorld(keyframe="stand")
        bez = Bez(sim)
        start = time.time()

        ctrl_dt = 0.02
        sim_dt = 0.002
        n_substeps = int(round(ctrl_dt / sim_dt))

        policy = OnnxController(
            policy_path=(_ONNX_DIR / "bez222_policy.onnx").as_posix(),
            # default_angles=np.array(sim.model.keyframe("home").qpos[7:]),
            ctrl_dt=ctrl_dt,
            n_substeps=n_substeps,
            action_scale=0.5,
            vel_scale_x=1.0,
            vel_scale_y=1.0,
            vel_scale_rot=1.0,
        )
        last_acc = np.array([0, 0, 0])
        last_vel = np.array([0, 0, 0])
        while sim.t < 3000:
            act = policy.get_control(sim.model, sim.data, bez.motor_control.get_q_legs(), bez.motor_control.get_qvel_legs())
            policy.cmd = [0, 0, 1]
            if policy._counter % policy._n_substeps == 0 and act is not None:
                bez.motor_control.set_left_leg_target_angles(act[0:6])
                bez.motor_control.set_right_leg_target_angles(act[6:])
                bez.motor_control.set_motor()
            imu_xmat = sim.data.site_xmat[sim.model.site("torso").id].reshape(3, 3)
            gravity = imu_xmat.T @ np.array([0, 0, -1])
            g2 = bez.sensors.get_pose().rotation_matrix.T @ np.array([0, 0, -1], dtype=np.float32)
            # print(f" gravity: {gravity}, g2: {g2}  EQ: {gravity==g2}")
            R_imu_to_world = bez.sensors.get_pose().rotation_matrix
            R_world_to_imu = R_imu_to_world.T
            g_world = np.array([1.52038368, -0.04286343,  9.89330155])
            g_world = np.array([0 ,0 , 9.81])

            linvel = sim.data.sensor("local_linvel").data
            acc = sim.data.sensor("accelerometer").data
            a_meas_world = R_imu_to_world @ acc
            a_lin_world = acc - g_world
            # dacc = (acc - last_acc)
            # linvel2 = last_vel +0.5 *(acc + last_acc)*sim.dt
            linvel2 = last_vel + a_lin_world * sim.dt
            print(f" acc: {acc}, a_lin_world: {a_lin_world} linvel2: {linvel2}")
            print(f" linvel: {linvel}, linvel2: {linvel2}  EQ: {np.isclose(linvel,linvel2)}")
            last_acc = a_lin_world
            # last_vel = linvel2
            sim.render(True)

            sim.step()

            elapsed = time.time() - start
            frames = sim.frame


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
        target_goal = [0.1, 0., 0, 10, 500]
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
        # walk.ready()
        # walk.world.step()
        # sim.wait(200)
        # walk.wait(100)
        # target_goal = [0.1, 0, 0.0, 10, 500]
        target_goal = Transformation(position=[0.07, -0.00, 0], euler=[0, 0, 0])
        ctrl_dt = 0.02
        sim_dt = 0.002
        n_substeps = int(round(ctrl_dt / sim_dt))

        policy = OnnxController(
            policy_path=(_ONNX_DIR / "bez222_policy.onnx").as_posix(),
            # default_angles=np.array(sim.model.keyframe("home").qpos[7:]),
            ctrl_dt=ctrl_dt,
            n_substeps=n_substeps,
            action_scale=0.5,
            vel_scale_x=1.0,
            vel_scale_y=1.0,
            vel_scale_rot=1.0,
        )
        start = time.time()
        x,y,theta = 0,0,0
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
                    print("here: ", x, y, theta)
                    print(f"ex: {x - bez.sensors.get_pose().position[0]} ey: {y - bez.sensors.get_pose().position[1]} etheta: {theta - bez.sensors.get_pose().orientation_euler[0]}")


            # walk.walk(target_goal, display_metrics=False)
            policy.cmd[0] = walk.dx*12.5
            policy.cmd[1] = walk.dy*12.5
            policy.cmd[2] = walk.dtheta*1

            act = policy.get_control(sim.model, sim.data, bez.motor_control.get_q_legs(),
                                     bez.motor_control.get_qvel_legs())
            if policy._counter % policy._n_substeps == 0 and act is not None:
                bez.motor_control.set_left_leg_target_angles(act[0:6])
                bez.motor_control.set_right_leg_target_angles(act[6:])
                bez.motor_control.set_motor()
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
        while sim.t < 300:
            sim.render(True)

            sim.step()
            # cv2.namedWindow("ball_cam", cv2.WINDOW_NORMAL)
            # cv2.imshow("ball_cam", bez.sensors.get_camera_image())
            # if cv2.waitKey(1) & 0xFF == 27:  # ESC
            #     break
            if sim.t > 10:
                print("dfs")
                print(sim.data.qpos)
            print(sim.data.qpos[2])
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
        # walk.ready()
        # sim.wait(500)
        # walk.world.step()
        ctrl_dt = 0.02
        sim_dt = 0.002
        n_substeps = int(round(ctrl_dt / sim_dt))

        policy = OnnxController(
            policy_path=(_ONNX_DIR / "bez222_policy.onnx").as_posix(),
            ctrl_dt=ctrl_dt,
            n_substeps=n_substeps,
            action_scale=0.5,
            vel_scale_x=1.0,
            vel_scale_y=1.0,
            vel_scale_rot=1.0,
        )
        start = time.time()
        x, y, theta = 0, 0, 0
        start = time.time()
        print("STARTING WALK")
        ball_pos = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        kicked = False
        ball_pixel = [0, 0]
        while sim.t < 300:
            if sim.frame % int((1/30.0) / sim.dt) == 0:
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

                        # print(
                        #     string,#end='\r',
                        #     flush=True,
                        # )

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
                policy.cmd[0] = walk.dx * 12.5
                policy.cmd[1] = walk.dy * 12.5
                policy.cmd[2] = walk.dtheta * 1
                act = policy.get_control(sim.model, sim.data, bez.motor_control.get_q_legs(),
                                         bez.motor_control.get_qvel_legs())
                if policy._counter % policy._n_substeps == 0 and act is not None:
                    bez.motor_control.set_left_leg_target_angles(act[0:6])
                    bez.motor_control.set_right_leg_target_angles(act[6:])
                    bez.motor_control.set_motor()
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

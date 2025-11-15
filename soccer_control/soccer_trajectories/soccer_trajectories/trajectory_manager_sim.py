#!/usr/bin/env python3

from sensor_msgs.msg import JointState
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.model.sim_world import SimWorld
from soccer_trajectories.trajectory_manager import TrajectoryManager

# from soccer_common import Transformation


class TrajectoryManagerSim(TrajectoryManager):
    """
    Interfaces with trajectory and sends to pybullet
    """

    def __init__(self, world: SimWorld, bez: Bez, robot_model: str, traj_name: str, mirror: bool = False):
        super(TrajectoryManagerSim, self).__init__(robot_model, traj_name, mirror)
        self.world = world
        self.bez = bez

    def read_joint_state(self) -> JointState:
        return JointState(name=list(self.bez.motor_control.ctrl_dofs_to_index.keys()), position=[0.0] * self.bez.motor_control.dof)

    def send_joint_msg(self, timestamp: float) -> None:
        joints, angles = self.trajectory.create_joint_states(timestamp)
        for i, joint in enumerate(joints):
            self.bez.motor_control.set_single_motor(joint,angles[i])

        self.bez.motor_control.set_motor()

    def send_trajectory(self, traj_name: str | None, mirror: bool = False) -> None:
        if traj_name is None:
            return

        self.process_trajectory(traj_name, mirror)

        t: float = 0
        while t <= self.trajectory.max_time:
            # print(f"t={t}, max {self.trajectory.max_time} .dt {self.world.dt}")
            try:
                self.send_joint_msg(t)
            except Exception as ex:
                exit(0)
            t += 1/100.0

            # print("time", t)
            # print(int((1/100.0)/ self.world.dt))
            for i in range(int((1/100.0)/ self.world.dt)): # TODO idk if i like this
                self.world.render(True)
                self.world.step()
            # self.world.render(True)
            # self.world.step()
            # time.sleep(0.01)
        # self.sim.ramp.close()
        # self.trajectory.reset()

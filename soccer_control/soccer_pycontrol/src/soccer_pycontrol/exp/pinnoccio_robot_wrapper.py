#
# In this short script, we show how to use RobotWrapper
# integrating different kinds of viewers
#
import os
from os.path import abspath, dirname, expanduser, join
from sys import argv

import numpy as np
import pinocchio
import pinocchio as pin
from numpy.linalg import norm, solve
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import GepettoVisualizer, MeshcatVisualizer

# from soccer_pycontrol.joints import Joints
# TODO should use pinnoccio as backend for rigid kinematics instead of pybullet. should be faster
VISUALIZER = MeshcatVisualizer

# Load the URDF model with RobotWrapper
# Conversion with str seems to be necessary when executing this file with ipython

urdf_model_path = expanduser("~") + "/catkin_ws/src/soccerbot/soccer_description/bez2_urdf_description/urdf/bez2_urdf_2.urdf"
mesh_dir = expanduser("~") + "/catkin_ws/src/soccerbot/soccer_description/bez2_urdf_description"

robot = RobotWrapper.BuildFromURDF(urdf_model_path, mesh_dir)
# alias
model = robot.model
data = robot.data


def fwd_kinematics() -> np.ndarray:
    q = [0] * 20  # pinocchio.randomConfiguration(model)
    # q = pinocchio.randomConfiguration(model)
    # for i in model.names:
    #     print(i, model.getJointId(i))
    # q[model.getJointId("right_leg_motor_0") - 1 : model.getJointId("right_leg_motor_5")] = [
    #     -4.440892098500626e-16,
    #     1.5707963267948966,
    #     1.5707963267948957,
    #     -3.141592653589793,
    #     1.5707963267948974,
    #     -1.5707963267948966,
    # ]
    q = np.array(q)
    # print("q: %s" % q.T)
    # print(model.getJointId("right_leg_motor_0"))
    # for i in model.names:
    #     print(i, model.getJointId(i))
    # pinocchio.forwardKinematics(model, data, q)
    # Print out the placement of each joint of the kinematic tree
    for name, oMi in zip(model.names, data.oMi):
        print(("{:<24} : {: .6f} {: .6f} {: .6f}".format(name, *oMi.translation.T.flat)))
    return q


def inverse_kinematics() -> np.ndarray:
    JOINT_ID = model.getJointId("right_leg_motor_5")
    ii = np.eye(3)  # np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    # oMdes = pinocchio.SE3(ii, np.array([0.0135, -0.035, -0.29]))
    oMdes = pinocchio.SE3(np.eye(3), np.array([0.0135, -0.035, -0.10289]))

    # # TODO try pink since this is doing a full body
    q = pinocchio.neutral(model)
    eps = 1e-4
    IT_MAX = 1000
    DT = 1e-1
    damp = 1e-12

    i = 0
    while True:
        pinocchio.forwardKinematics(model, data, q)
        iMd = data.oMi[JOINT_ID].actInv(oMdes)
        err = pinocchio.log(iMd).vector  # in joint frame
        if norm(err) < eps:
            success = True
            break
        if i >= IT_MAX:
            success = False
            break
        J = pinocchio.computeJointJacobian(model, data, q, JOINT_ID)  # in joint frame
        J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)
        v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
        q = pinocchio.integrate(model, q, v * DT)
        # if not i % 10:
        #     print('%d: error = %s' % (i, err.T))
        i += 1

    if success:
        print("Convergence achieved!")
    else:
        print("\nWarning: the iterative algorithm has not reached convergence to the desired precision")
    print("\nresult: %s" % q.flatten().tolist())
    print("\nfinal error: %s" % err.T)
    return q


if VISUALIZER:
    robot.setVisualizer(VISUALIZER())
    robot.initViewer()
    robot.loadViewerModel("pinocchio")
    import time

    s = time.time()

    q = fwd_kinematics()
    # v = pinocchio.utils.zero(model.nv)
    # pinocchio.ccrba(model, data, q, v)
    # # print(data.Ig)
    # print(pinocchio.centerOfMass(model, data, q))
    # print(time.time() - s)
    # q = inverse_kinematics()
    robot.display(q)

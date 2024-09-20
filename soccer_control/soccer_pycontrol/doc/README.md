| Class             | Responsibility                                                                                            |
| ----------------- | --------------------------------------------------------------------------------------------------------- |
| `Bez`             | High level abstraction to represent the model                                                             |
| `LoadModel`       | Interfaces with pybullet to load a pybullet model and set pose.                                           |
| `MotorControl`    | Class controls access to motor information and sets motor angles in pybullet                              |
| `Sensors`         | Interfaces with pybullet to extract sensor data.                                                          |
| `KinematicData`   | Class to contain all relevant information about the kinematics of model. Uses pinnoccio to load from urdf |
| `IKActions`       | Class for actions robot will do using ik                                                                  |
| `IKCalculation`   | Main inverse kinematic calculations.                                                                      |
| `Navigator`       | Main loop for the walk engine. Interfaces with all classes to perform walking.                            |
| `FootStepPlanner` | Class to interface with path for robot foot steps.                                                        |
| `Stabilize`       | Manages PID loops for pitch, roll while standing and walking.                                             |
| `StabilizePhase`  | Experimental for a phase roll pid thingy.                                                                 |
| `PybulletWorld`   | Class for interacting and managing with pybullet.                                                         |

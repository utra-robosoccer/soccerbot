| Class                  | Responsibility                                                 |
| ---------------------- | -------------------------------------------------------------- |
| `TrajectoryManager`    | Interfaces with trajectory and structure for following classes |
| `TrajectoryManagerRos` | Interfaces with trajectory and manages interaction with ROS    |
| `TrajectoryManagerSim` | Interfaces with trajectory and sends to pybullet.              |
| `Trajectory`           | Interpolates a CSV trajectory for multiple joints.             |
| `Pybullet`             | Sets up pybullet simulation for basic usage                    |

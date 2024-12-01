| Class                  | Responsibility                                                 |
| ---------------------- | -------------------------------------------------------------- |
| `TrajectoryManager`    | Interfaces with trajectory and structure for following classes |
| `TrajectoryManagerRos` | Interfaces with trajectory and manages interaction with ROS    |
| `TrajectoryManagerSim` | Interfaces with trajectory and sends to pybullet.              |
| `Trajectory`           | Interpolates a CSV trajectory for multiple joints.             |
| `Pybullet`             | Sets up pybullet simulation for basic usage                    |

localization
 - IPM

Navigation
 - global planner A*
 - Local Planner Pursuit
 - 

Mapping 
 - Create map
 - Add obstacles with a decay

Perception
 - segmentation
 - object localization - obj size, IPM
 - point cloud localization - IPM
 - Mapp & IOU classification comparision
 - Segmentation comparison

Strategy
 - Adding basic structure
 - porting old to new
 - Team communication
 - stratgey

Control
 - Muj= os.environ["ROS_NAMESPACE"]
        os.system(
            f"/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill {robot_ns}/soccer_strategy {robot_ns}/soccer_pycontrol {robot_ns}/soccer_trajectories'"
        )

        rospy.init_node("soccer_control")

        bez = BezROS()
        walker = NavigatorRos(bez)
        # walker.wait(50)
        # walker.ready()
        # bez.motor_control.set_motor()
        # walker.wait(50)
        # walker.goal_callback(PoseStamped())
        # walker.walk(d_x=0.04, t_goal=10)
        # target_goal = Transformation(position=[1, 0, 0], euler=[0, 0, 0])
        # walker.walk(target_goal)
        target_goal = [0.04, 0, 0, 10, 500]
        walker.walk(target_goal)

        walker.wait(100)oco
 - Placo tuning
 - COM balancing mode
   - Betteer balance way paper
 - Trajectory metrics
 - Refactoring
 - Trajectory generation
 - Adding in imu feedback
 - ZMP 
 - PID balancing walking
 - MPC
 - LQR

Infrastructure
 - Refactor dockerfile
 - Switch to ROS2
 - 

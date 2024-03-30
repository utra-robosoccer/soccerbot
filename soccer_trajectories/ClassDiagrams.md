# Documentation

###

```mermaid
%%{init: {"classDiagram": {"useMaxWidth": false}} }%%
classDiagram
    direction LR


    TrajectoryManager *--|> Pybullet
    TrajectoryManager *--|> Trajectory

    TrajectoryManagerRos --|>  TrajectoryManager


```

# Documentation

###

```mermaid
%%{init: {"classDiagram": {"useMaxWidth": false}} }%%
classDiagram
    direction LR


    TrajectoryManagerSim *--|> Pybullet
    Trajectory <|--* TrajectoryManager
    TrajectoryManager <|-- TrajectoryManagerSim
    TrajectoryManager <|--  TrajectoryManagerRos


```

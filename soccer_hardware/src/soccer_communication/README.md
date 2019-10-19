# soccer-communication
The PC side communication node for communicating with the microcontroller

### Required programs on the PC computer before running
```
pip3 install pyserial
pip install numpy --user
pip install prettytable --user
```
### To run the program on ROS
```
roslaunch soccer_hardware soccer_hardware.py
```

### Command-line arguments
There are currently 4 supported command-line arguments. All of them have defaults suitable to operation on the actual robot. You can use `python soccer_hardware.py --help` for the following information as well.
- `--ros` whether or not to import ROS dependencies and execute the logic used on the actual robot, or omit ROS dependencies and simply loop through a local static trajectory file
- `--port` the name of the virtual COM port associated with the microcontroller
- `--baud` the baud rate of the virtual COM port
- `--traj` name of the trajectory file to load at startup. Must be the name of the file, including ".csv" (note: not the path. The path to the trajectories folder is automatically prepended)

Example (similar to what it might look like for the embedded team):
```
python soccer_hardware.py --ros=False --port=COM7 --baud=230400 --traj=getupfront.csv
```

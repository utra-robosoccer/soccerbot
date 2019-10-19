# Control center

Simple python-flask REST server exposing remote process control apis for monitoring.

### Local Setup

#### 1. Install dependencies:

```
$ ./scripts/install.sh
```
#### 2. Set configurations:

Modify required keys in control_center/settings.py

#### 4. Run Monitoring development server:
```
$ catkin build soccer_control_center
$ roslaunch soccer_control_center default.launch
```
server will be running at http://localhost:8001

Or just use the executable
```
$ python app.py 
```

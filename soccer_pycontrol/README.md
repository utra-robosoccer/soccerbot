#### Control Code
The code here takes joint states from a robot and executes control code

IDE Setup
```shell script
Change your project python intepretor to the system python 3.6
Mark the folder soccer_pycontrol/src as a root directory
```

Installation
```shell script
# Make sure you are in the same directory as the README
sudo apt-get install python3-tk
python3 -m pip install --user virtualenv
virtualenv -p python3 venv
. venv/bin/activate
pip install -r requirements.txt

# In Pycharm intepretor to the virtual environment that has been created
# Settings > Project > Project Intepretor > Click on the Gear > Add > Virtualenv Environment > Existing Intepreter and enter this
# /path/to/workspace/soccer_pycontrol/venv/bin/python
# Mark the soccer_pycontrol/src as a source root directory by right clicking the src folder > mark as > sources root

cd src
python3 main.py
```

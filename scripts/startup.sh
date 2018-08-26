sleep 20
source ~/.bashrc
MY_IP=$(ifconfig wlan0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1')
export ROS_IP=$MY_IP
export ROS_MASTER_URI="http://"$ROS_IP":11311"
echo $ROS_MASTER_URI
roslaunch soccer_hardware soccer_hardware.launch
read -p "Press any key to continue... " -n1 -s

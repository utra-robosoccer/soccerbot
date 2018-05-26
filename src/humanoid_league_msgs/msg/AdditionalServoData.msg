# This message provides additional data from the servos, which is not included in JointState.msg
# Should mainly used for monitoring and debug purposes

# Setting the value to -1 means there is no data from this motor
float32[] voltage
sensor_msgs/Temperature[] temperature
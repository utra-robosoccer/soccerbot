# Comm.py

import time
import os
import numpy as np
from prettytable import PrettyTable
from Transmitter import Transmitter
from Receiver import Receiver
from utility import *

try:
    import rospy
    from soccer_msgs.msg import RobotGoal
    from soccer_msgs.msg import RobotState
    from sensor_msgs.msg import Imu
    from transformations import *
except ImportError:
    pass

class Comm:
    def __init__(self):
        self.first = True
        
    def start_up(self, ser, ros_is_on, traj, step_is_on):
        self.last_print_time = time.time()
        
        if(not self.first):
            self.ser = ser
            self.tx.change_port(ser)
            self.rx.change_port(ser)
            return
        
        self.first = False
        self.ros_is_on = ros_is_on
        self.step_is_on = step_is_on
        self.use_trajectory = False
        
        self.tx = Transmitter(ser)
        self.rx = Receiver(ser, ros_is_on)
        
        if(self.ros_is_on == True):
            rospy.init_node('soccer_hardware', anonymous=True)
            rospy.Subscriber("robotGoal", RobotGoal, self.trajectory_callback, queue_size=1)
            self.rx.pub = rospy.Publisher('soccerbot/imu', Imu, queue_size=1)
            self.rx.pub2 = rospy.Publisher('soccerbot/robotState', RobotState, queue_size=1)
        else:
            trajectories_dir = os.path.join("trajectories", traj)
            try:
                self.trajectory = np.loadtxt(
                    open(trajectories_dir, "rb"),
                    delimiter=",",
                    skiprows=0
                )
                
                logString("Opened trajectory {0}".format(traj))
                self.use_trajectory = True
            except IOError as err:
                logString("Error: Could not open trajectory: {0}".format(err))
                logString("(Is your shell running in the soccer-communication directory?)")
                logString("Standing pose will be sent instead...")

    def print_angles(self, sent, received):
        ''' Prints out 2 numpy vectors side-by-side, where the first vector entry
            is interpreted as belonging to motor 1, the seconds to motor 2, etc.
        '''
        assert sent.shape[0] == received.shape[0]
        t = PrettyTable(['Motor Number', 'Sent', 'Received'])
        
        for i in range(sent.shape[0]):
            t.add_row([str(i + 1), round(sent[i][0], 4), round(received[i][0], 2)])
        
        print(t)
    
    def print_imu(self, received):
        ''' Prints out a numpy vector interpreted as data from the IMU, in the
            order X-gyro, Y-gyro, Z-gyro, X-accel, Y-accel, Z-accel.
        '''
        
        t = PrettyTable(['', 'Gyro (deg/s)', 'Accel (m/s^2)'])
        
        t.add_row(["X", round(received[0][0], 2), round(received[3][0], 2)])
        t.add_row(["Y", round(received[1][0], 2), round(received[4][0], 2)])
        t.add_row(["Z", round(received[2][0], 2), round(received[5][0], 2)])
        
        print(t)

    def print_handler(self, goal_angles):
        current_time = time.time()
        if(current_time - self.last_print_time >= 1):
            self.last_print_time = current_time
            print('\n')
            logString("Received: {0}".format(self.rx.num_receptions))
            logString("Transmitted: {0}\n".format(self.tx.num_transmissions))
            if(self.rx.num_receptions > 0):
                # Prints the last valid data received
                self.print_angles(goal_angles[0:12], self.rx.received_angles[0:12])
                self.print_imu(self.rx.received_imu)
    
    def communicate(self, goal_angles):
        self.tx.transmit(goal_angles)
        self.rx.receive()
        self.print_handler(goal_angles)
    
    def trajectory_callback(self, robotGoal):
        '''
        Used by ROS. Converts the motor array from the order and sign convention
        used by controls to that used by embedded
        '''
        m = getCtrlToMcuAngleMap()
        goalangles = m.dot(robotGoal.trajectories[0:18])
        goalangles = goalangles[:, np.newaxis]
        self.communicate(goalangles)
    
    def begin_event_loop(self):
        if(self.ros_is_on == True):
            rospy.spin() 
        else:
            while(True):
                if(self.use_trajectory):
                    # Iterate through the static trajectory forever
                    # TODO: If the static trajectories are ever re-generated, we will need
                    # TODO: to dot the trajectories with the ctrlToMcuAngleMap to shuffle
                    # TODO: them properly
                    for i in range(self.trajectory.shape[1]):
                        if(self.step_is_on):
                            input('Press enter to send next pose')
                        
                        goal_angles = self.trajectory[:, i:i+1]
                        self.communicate(goal_angles)
                else:
                     # Send standing pose
                    if(self.step_is_on):
                        input('Press enter to send next pose')

                    self.communicate(np.zeros((18,1)))

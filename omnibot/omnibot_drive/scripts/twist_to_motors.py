#!/usr/bin/python2.7

import rospy
import roslib
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math

class TwistToMotors():
    def __init__(self):
        rospy.init_node("twist_to_motors")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)

        # Parameters
        self.wheeldiagonal = rospy.get_param("~base_width", 0.432)
        self.wheelstraight = math.sqrt(self.wheeldiagonal)
        self.wheelradius = rospy.get_param("~wheel_radius", 0.09)
        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)

        # Publishers and Subscribers
        self.pub_nemotor = rospy.Publisher('/omnibot/right_front_wheel_hinge_controller/command', Float64, queue_size=10)
        self.pub_nwmotor = rospy.Publisher('/omnibot/left_front_wheel_hinge_controller/command', Float64, queue_size=10)
        self.pub_semotor = rospy.Publisher('/omnibot/right_back_wheel_hinge_controller/command', Float64, queue_size=10)
        self.pub_swmotor = rospy.Publisher('/omnibot/left_back_wheel_hinge_controller/command', Float64, queue_size=10)
        rospy.Subscriber('/omnibot/cmd_vel', Twist, self.twistCallback)

    def spin(self):
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks

        while not rospy.is_shutdown():
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()

    def negroot(self, num):
        if num >= 0:
            return math.sqrt(num)
        else:
            return -math.sqrt(math.fabs(num))

    def spinOnce(self):

        ne = self.negroot(self.dx - self.dy) + self.dr * self.wheeldiagonal / 2
        nw = self.negroot(self.dx + self.dy) - self.dr * self.wheeldiagonal / 2
        se = self.negroot(self.dx + self.dy) + self.dr * self.wheeldiagonal / 2
        sw = self.negroot(self.dx - self.dy) - self.dr * self.wheeldiagonal / 2

        self.pub_nemotor.publish(ne)
        self.pub_nwmotor.publish(nw)
        self.pub_semotor.publish(se)
        self.pub_swmotor.publish(sw)

        self.ticks_since_target += 1

    def twistCallback(self,msg):
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y

if __name__ == '__main__':
    """ main """
    try:
        twistToMotors = TwistToMotors()
        twistToMotors.spin()
    except rospy.ROSInterruptException:
        pass

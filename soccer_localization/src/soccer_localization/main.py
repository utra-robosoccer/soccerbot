import rospy

from soccer_localization.field_lines_ukf import FieldLinesUKF

if __name__ == "__main__":
    f = FieldLinesUKF()
    rospy.spin()

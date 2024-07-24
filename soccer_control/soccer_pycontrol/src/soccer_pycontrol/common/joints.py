import enum


class Joints(enum.IntEnum):
    """
    The list of joints of the robot `Joint Table <https://docs.google.com/spreadsheets/d/1KgIYwm3fNen8yjLEa-FEWq-GnRUnBjyg4z64nZ2uBv8/edit#gid=0>`_
    """

    LEFT_ARM_1 = 0
    LEFT_ARM_2 = 1
    RIGHT_ARM_1 = 2
    RIGHT_ARM_2 = 3
    LEFT_LEG_1 = 4
    LEFT_LEG_2 = 5
    LEFT_LEG_3 = 6
    LEFT_LEG_4 = 7
    LEFT_LEG_5 = 8
    LEFT_LEG_6 = 9
    RIGHT_LEG_1 = 10
    RIGHT_LEG_2 = 11
    RIGHT_LEG_3 = 12
    RIGHT_LEG_4 = 13
    RIGHT_LEG_5 = 14
    RIGHT_LEG_6 = 15
    HEAD_1 = 16  # Left Right (Yaw)
    HEAD_2 = 17  # Up Down (Pitch)


print([e.name for e in Joints])

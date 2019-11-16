std_msgs/Header header

# command message enumeration
int8 IDLE=0
int8 GO_TO_LOCATION=1
int8 GO_AFTER_BALL=2


# leg message enumeration
int8 LEFT_LEG=0
int8 RIGHT_LEG=1

# message parameters sent
int8 command
geometry_msgs/Pose2D destPose
geometry_msgs/Pose2D currPose
geometry_msgs/Pose2D destBall
int8 leg
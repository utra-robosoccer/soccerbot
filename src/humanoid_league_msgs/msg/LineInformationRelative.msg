# Contains all relative information about line features on the field

# The header is included to get the time stamp for later use in tf
std_msgs/Header header

LineSegmentRelative[] segments
LineIntersectionRelative[] markings
LineCircleRelative[] circles
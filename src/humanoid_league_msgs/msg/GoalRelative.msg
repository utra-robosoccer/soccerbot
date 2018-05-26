# Relative position to a goal

# The header is included to get the time stamp for later use in tf
std_msgs/Header header

# Position of the left goal post feet (in meter)
geometry_msgs/Point left_post

# Position of the right post, null if only one post was seen
geometry_msgs/Point right_post

# Vector pointing to the (probable) center of the goal (in meters).
# Should only be used if only one goal post is visible. If both are visible this should be none.
# This is normally an educated guess, using the goal bar or the position of the post on the image
geometry_msgs/Point center_direction

# A certainty rating between 0 and 1, where 1 is the surest.
# 0 means no goal was found
float32 confidence
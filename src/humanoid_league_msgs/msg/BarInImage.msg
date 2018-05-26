# A (possible) goal bar in the image. It is defined by the two end points and a width.

# Two points defining the significant axis of the post
geometry_msgs/Point left_point
geometry_msgs/Point right_point
# Orthogonal to significant vector (in pixel)
float32 width

# A certainty rating between 0 and 1, where 1 is the surest.
float32 confidence
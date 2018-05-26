# Providing a (possible) ball in the image

# Center point of the ball, the z-axis should be ignored (in pixel)
geometry_msgs/Point center
# Diameter of the ball (in pixel)
float64 diameter

# A certainty rating between 0 and 1, where 1 is the surest.
# 0 means no ball was found
float32 confidence

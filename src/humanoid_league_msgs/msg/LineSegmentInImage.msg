# A normal line segment in the image

# Two points defining the vector of the line. The center is orthogonally in the middle of the line
geometry_msgs/Point start
geometry_msgs/Point end

# Orthogonal to the significant vector
float32 start_width
float32 end_with

# A certainty rating between 0 and 1, where 1 is the surest.
float32 confidence
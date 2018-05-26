# A line intersection feature in the image

# The type defines which kind of intersection is present
uint8 UNDEFINED=0
uint8 L=1
uint8 T=2
uint8 X=3
uint8 type

# The line segments related to this crossing
humanoid_league_msgs/LineSegmentInImage segments

# A certainty rating between 0 and 1, where 1 is the surest.
float32 confidence

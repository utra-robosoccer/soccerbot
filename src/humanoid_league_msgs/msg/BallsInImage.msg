# Contains multiple balls on an image. Should be mainly used to provide ball candidates (for example round shapes)
# in the first step of the vision pipeline.

# The header is included to get the time stamp for later use in tf
std_msgs/Header header

# An empty array means no balls have been found.
humanoid_league_msgs/BallInImage[] candidates
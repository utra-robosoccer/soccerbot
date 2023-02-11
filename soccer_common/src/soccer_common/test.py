import numpy as np

from soccer_common.transformation import Transformation

# Probably need to update this angle based on the side we are on pi/2.0
tf_robot_to_field = Transformation(pos_theta=np.array([0.0, 0.0, np.pi / 2.0 + -0.15315323112995854]))
# print(f"Robot yaw: {robot_yaw}")
print(f"Matrix:\n{tf_robot_to_field.rotation_matrix}")
goal_post_coords = [4.655318254906669, 3.5303345309879197, 0.0]
goal_robot_frame = np.array([[4.655318254906669], [3.5303345309879197], [0.0]])
goal_world_frame = np.array([[-4.5], [1.3], [0.0]])
print(f"Goal robot frame\n{goal_robot_frame}")
print(f"Goal world frame\n{goal_world_frame}")
position_from_post = goal_world_frame - np.dot(tf_robot_to_field.rotation_matrix, goal_robot_frame)
print(f"Estimated Robot position:\n{position_from_post}")

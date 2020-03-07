% General World parameters
density = 1000;

% Field parameters
plane_dim = [9 6 0.09];

% Contact/friction parameters
contact_stiffness = 3000;
contact_damping = 500;
mu_k = 0.6;
mu_s = 0.8;
mu_vth = 0.1;
contact_point_radius = 1e-4;

% Foot Parameters
cleat_length = 0.00;
foot_box = [0.09 0.07 0.01474 + cleat_length];
right_collision_center = [0.00385 0.00401 -0.00737];

right_foot_to_corner_1 = right_collision_center + [foot_box(1)/2 foot_box(2)/2 -foot_box(3)];
right_foot_to_corner_2 = right_collision_center + [foot_box(1)/2 -foot_box(2)/2 -foot_box(3)];
right_foot_to_corner_3 = right_collision_center + [-foot_box(1)/2 -foot_box(2)/2 -foot_box(3)];
right_foot_to_corner_4 = right_collision_center + [-foot_box(1)/2 foot_box(2)/2 -foot_box(3)];

left_collision_center = right_collision_center;
left_collision_center(2) = -left_collision_center(2);
left_foot_to_corner_1 = left_collision_center + [foot_box(1)/2 foot_box(2)/2 -foot_box(3)];
left_foot_to_corner_2 = left_collision_center + [foot_box(1)/2 -foot_box(2)/2 -foot_box(3)];
left_foot_to_corner_3 = left_collision_center + [-foot_box(1)/2 -foot_box(2)/2 -foot_box(3)];
left_foot_to_corner_4 = left_collision_center + [-foot_box(1)/2 foot_box(2)/2 -foot_box(3)];

% Motion Parameters
hip_height = 0.165;

% Motor Parameters
% http://emanual.robotis.com/docs/en/dxl/mx/mx-28/
motor_max_speed = 50/60; % 50 revolutions per minute
motor_p = 128 / 8;
motor_i = 0 * 1000 / 2048;
motor_d = 0 * 4 / 1000;
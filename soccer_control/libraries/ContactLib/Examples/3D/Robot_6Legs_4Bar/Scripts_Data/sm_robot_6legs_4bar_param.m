% Parameters for sm_robot_6legs_4bar.slx
% Copyright 2018-2019 The MathWorks, Inc.

% Robot dimensions
frame_len   = 23.6e-2;  % m 
frame_wid   = 5.0e-2; % m
frame_xhole = 5.9e-2; % m

link_th    = 3.0e-3; % m
link_h     = 1.5e-2; % m
rad_hole   = 2e-3;   % m

leg_l      = 8.5e-2; % m
fol_l      = 4.25e-2;% m
cam_l      = 1e-2;   % m

part_rho   = 1000;  %kg/m^3

% Colors
fol_clr = [0.6 1.0 0.6];
cam_clr = [1.0 0.8 0.6];
leg_clr = [0 0.4 0.6];
fra_clr = [1 1 1]*0.4;
pin_clr = [1 1 1]*0.4;
flr_clr = [1 1 1]*0.95;

% Contact and friction parameters
flr_k = 1e2;
flr_b = 1e1;
flr_muk = 0.7;
flr_mus = 0.9;
flr_vth = 0.01;

% Floor dimensions
flr1_len = 1;
flr1_wid = 0.5;
flr1_dep = 0.01;
flr1_ang = 0;  % deg, rel to world

flr2_len = flr1_len/8;
flr2_wid = flr1_wid;
flr2_dep = flr1_dep;
flr2_ang = 10; % deg, rel to World

flr3_len = flr1_len/3;
flr3_wid = flr1_wid;
flr3_dep = flr1_dep;
flr3_ang = 0;  % deg, rel to World

% Visualize Surfaces
flr_dep_pcnt = 0.1;  %(0-1)


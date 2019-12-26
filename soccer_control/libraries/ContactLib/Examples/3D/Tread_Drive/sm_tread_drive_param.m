% Parameters for sm_tread_drive
% Copyright 2017-2019 The MathWorks, Inc.

% Tread geometry
tread_w = 0.6;
tread_l = 2.8;
tread_h = tread_l/4;
tread_l = 2.8;
tread_roller_rad = tread_h/2*0.8;
tread_rad = tread_roller_rad*1.05;
tread_xlen = 0.25;
tread_clr = [0.95 0.95 0.6];
roller_clr = [1 1 1]*0.8;
tread_sep = 1.8;

% Ground geometry
ground_sph_rad = 0.6;
ground_sph_rad_bump = ground_sph_rad*1.3;
ground_clr = [1 1 1]*0.8;
ground_opc = 1;
ground_x = 12;
ground_y = 10;
ground_z = ground_sph_rad;

% Ground contact force
ground_k = 5e6;
ground_b = 1e7;

% Tread friction force
tread_muk = 0.7;
tread_mus = 0.9;
tread_vth = 0.001;


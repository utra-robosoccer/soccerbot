% Code to plot simulation results from Coll3D_04_Ball_in_Tube_Fixed
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Coll3D_04_Ball_in_Tube_Fixed)
catch
    h1_Coll3D_04_Ball_in_Tube_Fixed=figure('Name','Coll3D_04_Ball_in_Tube_Fixed');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Coll3D_04_Ball_in_Tube_Fixed','var'))
    sim('Coll3D_04_Ball_in_Tube_Fixed')
end

% Get tube dimensions from model workspace
temp_hws = get_param(bdroot,'modelworkspace');
temp_tube = temp_hws.getVariable('tube');
temp_ball = temp_hws.getVariable('ball');


% Get simulation results
temp_bpx = simlog_Coll3D_04_Ball_in_Tube_Fixed.x6_DOF_Joint.Px.p.series.values;
temp_bpy = simlog_Coll3D_04_Ball_in_Tube_Fixed.x6_DOF_Joint.Py.p.series.values;
temp_bpz = simlog_Coll3D_04_Ball_in_Tube_Fixed.x6_DOF_Joint.Pz.p.series.values;

% Get initial ball position

% Plot results
plot3(-temp_bpz,-temp_bpx,temp_bpy,'r','LineWidth',2);
grid on
box on
hold on

% Plot tube
[temp_tubex, temp_tubey, temp_tubez] = cylinder([1 1],100);
tube_h = surf(temp_tubez*temp_tube.length-temp_tube.length/2,temp_tubex*temp_tube.inner_rad,temp_tubey*temp_tube.inner_rad);
set(tube_h,'EdgeColor','none','DiffuseStrength',1,'AmbientStrength',1,'FaceColor',[0.0 0.5 0.7],'FaceAlpha',0.7);

% Plot ball
[temp_ballx, temp_bally, temp_ballz] = sphere(100);
ball_h = surf(...
    temp_ballx*temp_ball.rad-temp_bpz(1),...
    temp_bally*temp_ball.rad-temp_bpx(1),...
    temp_ballz*temp_ball.rad+temp_bpy(1));
set(ball_h,'EdgeColor','none','AmbientStrength',1,'FaceColor',[0.8 0.0 0.2],'FaceAlpha',0.4);

lightangle(-45,30)

hold off
axis equal
%axis([[-1 1]*temp_tube.length/2*2 [-1 1 -1 1]*temp_tube.outer_rad])
title('Ball Position in Tube (m)');

% Remove temporary variables
clear temp_bpx temp_bpy temp_bpz tube_h temp_cylx temp_cyly temp_cylz


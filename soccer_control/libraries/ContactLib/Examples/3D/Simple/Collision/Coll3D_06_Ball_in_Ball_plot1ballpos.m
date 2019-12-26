% Code to plot simulation results from Coll3D_06_Ball_in_Ball
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Coll3D_06_Ball_in_Ball)
catch
    h1_Coll3D_06_Ball_in_Ball=figure('Name','Coll3D_06_Ball_in_Ball');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Coll3D_06_Ball_in_Ball','var'))
    sim('Coll3D_06_Ball_in_Ball')
end

% Get simulation results
temp_bpx = simlog_Coll3D_06_Ball_in_Ball.x6_DOF_Joint.Px.p.series.values;
temp_bpy = simlog_Coll3D_06_Ball_in_Ball.x6_DOF_Joint.Py.p.series.values;
temp_bpz = simlog_Coll3D_06_Ball_in_Ball.x6_DOF_Joint.Pz.p.series.values;

% Get ball dimensions from model workspace
temp_hws = get_param(bdroot,'modelworkspace');
temp_outer_ball = temp_hws.getVariable('outer_ball');
temp_inner_ball = temp_hws.getVariable('inner_ball');

% Plot results
plot3(temp_bpx,temp_bpy,temp_bpz,'r','LineWidth',2);
grid on
box on
hold on

% Plot ball
[temp_ballx, temp_bally, temp_ballz] = sphere(100);
outerball_h = surf(...
    temp_ballx*temp_outer_ball.inner_rad,...
    temp_bally*temp_outer_ball.inner_rad,...
    temp_ballz*temp_outer_ball.inner_rad);
set(outerball_h,'EdgeColor','none','AmbientStrength',1,'FaceColor',[0.0 0.5 0.7],'FaceAlpha',0.4);

ball_h = surf(...
    temp_ballx*temp_inner_ball.rad+temp_bpx(1)*ones(size(temp_ballx)),...
    temp_bally*temp_inner_ball.rad+temp_bpy(1)*ones(size(temp_bally)),...
    temp_ballz*temp_inner_ball.rad+temp_bpz(1)*ones(size(temp_ballz)));
set(ball_h,'EdgeColor','none','AmbientStrength',1,'FaceColor',[0.8 0.5 0.2],'FaceAlpha',0.4);

lightangle(45,30)

hold off
axis equal
title('Ball Position in Ball');

% Remove temporary variables
clear temp_bpx temp_bpy temp_bpz tube_h temp_cylx temp_cyly temp_cylz
clear temp_inner_ball temp_outer_ball temp_hws

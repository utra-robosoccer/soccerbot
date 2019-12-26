% Code to plot simulation results from Coll3D_08_Ball_in_Spinning_Cone
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Coll3D_08_Ball_in_Spinning_Cone)
catch
    h1_Coll3D_08_Ball_in_Spinning_Cone=figure('Name','Coll3D_08_Ball_in_Spinning_Cone');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Coll3D_08_Ball_in_Spinning_Cone','var'))
    sim('Coll3D_08_Ball_in_Spinning_Cone')
end

% Get tube dimensions from model workspace
temp_hws = get_param(bdroot,'modelworkspace');
temp_cone = temp_hws.getVariable('cone');
temp_ball = temp_hws.getVariable('ball');

% Get simulation results
temp_bpx_in = simlog_Coll3D_08_Ball_in_Spinning_Cone.x6_DOF_Joint.Px.p.series.values;
temp_bpy_in = simlog_Coll3D_08_Ball_in_Spinning_Cone.x6_DOF_Joint.Py.p.series.values;
temp_bpz_in = simlog_Coll3D_08_Ball_in_Spinning_Cone.x6_DOF_Joint.Pz.p.series.values;

% Plot results
plot3(-temp_bpy_in,-temp_bpx_in,-temp_bpz_in,'r','LineWidth',2);
grid on
box on
hold on

temp_angle_vector = linspace(0,2*pi,100);
h_out = min(temp_cone.h,temp_cone.or*tand(temp_cone.angle));
h_in = min(h_out,temp_cone.ir*tand(temp_cone.angle));
x_in = max(0,temp_cone.ir-temp_cone.h/tand(temp_cone.angle));

plot3(sin(temp_angle_vector)*temp_cone.ir,...
    cos(temp_angle_vector)*temp_cone.ir,zeros(1,length(temp_angle_vector)),...
    'Color',[0 0.4 0.6],'LineWidth',3);

plot3(sin(temp_angle_vector)*x_in,...
    cos(temp_angle_vector)*x_in,-h_in*ones(1,length(temp_angle_vector)),...
    'Color',[0 0.4 0.6],'LineWidth',3);
view(-37,11);


% Plot ball
[temp_ballx, temp_bally, temp_ballz] = sphere(100);
ball_h = surf(...
    temp_bally*temp_ball.rad-temp_bpy_in(1),...
    temp_ballx*temp_ball.rad-temp_bpx_in(1),...
    -(temp_ballz*temp_ball.rad+temp_bpz_in(1)));
set(ball_h,'EdgeColor','none','AmbientStrength',1,'FaceColor',[0.8 0.0 0.2],'FaceAlpha',0.4);

lightangle(-45,30)

hold off
axis equal
title('Ball Position in Cone (m)');

% Remove temporary variables
clear temp_bpx_in temp_bpy_in temp_bpz_in 
clear temp_ballx temp_bally temp_ballz
clear temp_hws temp_ball temp_cone
clear temp_angle_vector 
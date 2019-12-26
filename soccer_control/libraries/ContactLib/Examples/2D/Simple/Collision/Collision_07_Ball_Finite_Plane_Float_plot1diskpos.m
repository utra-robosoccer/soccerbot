% Code to plot simulation results from Collision_07_Ball_Finite_Plane_Float
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Collision_07_Ball_Finite_Plane_Float)
catch
    h1_Collision_07_Ball_Finite_Plane_Float=figure('Name','Collision_07_Ball_Finite_Plane_Float');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Collision_07_Ball_Finite_Plane_Float','var'))
    sim('Collision_07_Ball_Finite_Plane_Float')
end

% Get simulation results
simlog_plax = simlog_Collision_07_Ball_Finite_Plane_Float.Planar_Joint_Plane.Px.p.series.values;
simlog_play = simlog_Collision_07_Ball_Finite_Plane_Float.Planar_Joint_Plane.Py.p.series.values;
simlog_plarz = simlog_Collision_07_Ball_Finite_Plane_Float.Planar_Joint_Plane.Rz.q.series.values('rad');

temp_bxyz = logsout_Collision_07_Ball_Finite_Plane_Float.get('Ball_xyz');

% Plot results
temp_colorOrder = get(gca,'DefaultAxesColorOrder');
patch(sin(0:0.1:2*pi)*0.05+temp_bxyz.Values.Data(1,1),cos(0:0.1:2*pi)*0.05+temp_bxyz.Values.Data(1,3),temp_colorOrder(2,:),'FaceAlpha',0.2)
hold on
plot([simlog_plax(1)-0.5*cos(simlog_plarz(1)) simlog_plax(1)+0.5*cos(simlog_plarz(1))],...
    [simlog_play(1)-0.5*sin(simlog_plarz(1)) simlog_play(1)+0.5*sin(simlog_plarz(1))],...
    'Color',[1 1 1]*0.7,'LineWidth',5);
plot(simlog_plax,simlog_play,'Color',[1 1 1]*0.7,'LineWidth',1);
plot([simlog_plax(end)-0.5*cos(simlog_plarz(end)) simlog_plax(end)+0.5*cos(simlog_plarz(end))],...
    [simlog_play(end)-0.5*sin(simlog_plarz(end)) simlog_play(end)+0.5*sin(simlog_plarz(end))],...
    'Color',[1 1 1]*0.2,'LineWidth',5);
plot(temp_bxyz.Values.Data(:,1),temp_bxyz.Values.Data(:,3),'Color',temp_colorOrder(2,:),'LineWidth',1);

patch(sin(0:0.1:2*pi)*0.05+temp_bxyz.Values.Data(end,1),cos(0:0.1:2*pi)*0.05+temp_bxyz.Values.Data(end,3),temp_colorOrder(2,:))

hold off
xlabel('Horizontal Position (m)');
ylabel('Vertical Position (m)');
title('Position of Disk and Plane');
grid on
axis square
axis equal

% Remove temporary variables
clear simlog_plax simlog_play simlog_plarz 
clear temp_bxyz temp_colorOrder




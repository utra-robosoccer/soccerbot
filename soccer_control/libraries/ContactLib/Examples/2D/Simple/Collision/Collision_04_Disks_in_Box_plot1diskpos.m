% Code to plot simulation results from Collision_04_Disks_in_Box
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Collision_04_Disks_in_Box)
catch
    h1_Collision_04_Disks_in_Box=figure('Name','Collision_04_Disks_in_Box');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Collision_04_Disks_in_Box','var'))
    sim('Collision_04_Disks_in_Box')
end

% Get simulation results
simlog_d1x = simlog_Collision_04_Disks_in_Box.Planar_Joint_Disk1.Px.p.series.values;
simlog_d1y = simlog_Collision_04_Disks_in_Box.Planar_Joint_Disk1.Py.p.series.values;
simlog_d2x = simlog_Collision_04_Disks_in_Box.Planar_Joint_Disk2.Px.p.series.values;
simlog_d2y = simlog_Collision_04_Disks_in_Box.Planar_Joint_Disk2.Py.p.series.values;
simlog_d3x = simlog_Collision_04_Disks_in_Box.Planar_Joint_Disk3.Px.p.series.values;
simlog_d3y = simlog_Collision_04_Disks_in_Box.Planar_Joint_Disk3.Py.p.series.values;

% Plot results
temp_colorOrder = get(gca,'DefaultAxesColorOrder');
plot(simlog_d1y,simlog_d1x,'LineWidth',1);
hold on
plot(simlog_d2y,simlog_d2x,'LineWidth',1);
plot(simlog_d3y,simlog_d3x,'LineWidth',1);
plot([-1 1 1 -1 -1 1]*0.5,[-1 -1 1 1 -1 -1]*0.5,'Color',[0.5 0.5 0.5],'LineWidth',5);
patch(sin(0:0.1:2*pi)*0.05+simlog_d1y(1),cos(0:0.1:2*pi)*0.05+simlog_d1x(1),temp_colorOrder(1,:),'FaceAlpha',0.2)
patch(sin(0:0.1:2*pi)*0.05+simlog_d2y(1),cos(0:0.1:2*pi)*0.05+simlog_d2x(1),temp_colorOrder(2,:),'FaceAlpha',0.2)
patch(sin(0:0.1:2*pi)*0.05+simlog_d3y(1),cos(0:0.1:2*pi)*0.05+simlog_d3x(1),temp_colorOrder(3,:),'FaceAlpha',0.2)

patch(sin(0:0.1:2*pi)*0.05+simlog_d1y(end),cos(0:0.1:2*pi)*0.05+simlog_d1x(end),temp_colorOrder(1,:))
patch(sin(0:0.1:2*pi)*0.05+simlog_d2y(end),cos(0:0.1:2*pi)*0.05+simlog_d2x(end),temp_colorOrder(2,:))
patch(sin(0:0.1:2*pi)*0.05+simlog_d3y(end),cos(0:0.1:2*pi)*0.05+simlog_d3x(end),temp_colorOrder(3,:))

hold off
xlabel('Horizontal Position (m)');
ylabel('Vertical Position (m)');
title('Disk Position (m)');
axis equal
axis square
axis([-1 1 -1 1]*0.6)

% Remove temporary variables
clear simlog_d1y simlog_d1x temp_colorOrder
clear simlog_d2y simlog_d2x
clear simlog_d3y simlog_d3x



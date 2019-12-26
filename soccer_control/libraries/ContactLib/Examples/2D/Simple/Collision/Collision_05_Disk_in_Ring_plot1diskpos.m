% Code to plot simulation results from Collision_05_Disk_in_Ring
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Collision_05_Disk_in_Ring)
catch
    h1_Collision_05_Disk_in_Ring=figure('Name','Collision_05_Disk_in_Ring');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Collision_05_Disk_in_Ring','var'))
    sim('Collision_05_Disk_in_Ring')
end

% Get simulation results
simlog_d1x = simlog_Collision_05_Disk_in_Ring.Planar_Joint.Px.p.series.values;
simlog_d1y = simlog_Collision_05_Disk_in_Ring.Planar_Joint.Py.p.series.values;

% Plot results
temp_colorOrder = get(gca,'DefaultAxesColorOrder');
plot(simlog_d1x,simlog_d1y,'Color',temp_colorOrder(2,:),'LineWidth',1);
hold on
plot(sin(0:0.1:2.2*pi)*0.21,cos(0:0.1:2.2*pi)*0.21,'Color',[0.5 0.5 0.5],'LineWidth',5);
patch(sin(0:0.1:2*pi)*0.05+simlog_d1x(1),cos(0:0.1:2*pi)*0.05+simlog_d1y(1),temp_colorOrder(2,:),'FaceAlpha',0.2)
patch(sin(0:0.1:2*pi)*0.05+simlog_d1x(end),cos(0:0.1:2*pi)*0.05+simlog_d1y(end),temp_colorOrder(2,:))

hold off
xlabel('Horizontal Position (m)');
ylabel('Vertical Position (m)');
title('Disk Position (m)');
axis equal
axis square
axis([-1 1 -1 1]*0.25)

% Remove temporary variables
clear simlog_d1y simlog_d1x temp_colorOrder




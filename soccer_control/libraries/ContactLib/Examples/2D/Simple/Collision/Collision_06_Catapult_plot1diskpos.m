% Code to plot simulation results from Collision_06_Catapult
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Collision_06_Catapult)
catch
    h1_Collision_06_Catapult=figure('Name','Collision_06_Catapult');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Collision_06_Catapult','var'))
    sim('Collision_06_Catapult')
end

% Get simulation results
temp_bxyz = logsout_Collision_06_Catapult.get('Disk_xyz');

% Plot results
temp_colorOrder = get(gca,'DefaultAxesColorOrder');
plot(temp_bxyz.Values.Data(:,1),temp_bxyz.Values.Data(:,3),'LineWidth',1);
hold on
plot([-0.50 1 1],[0 0 1]-0.15,'Color',[1 1 1]*0.5,'LineWidth',3);
patch(sin(0:0.1:2*pi)*0.05+temp_bxyz.Values.Data(1,1),cos(0:0.1:2*pi)*0.05+temp_bxyz.Values.Data(1,3),temp_colorOrder(1,:),'FaceAlpha',0.2)
patch(sin(0:0.1:2*pi)*0.05+temp_bxyz.Values.Data(end,1),cos(0:0.1:2*pi)*0.05+temp_bxyz.Values.Data(end,3),temp_colorOrder(1,:))
hold off

grid on
xlabel('Horizontal Position (m)');
ylabel('Vertical Position (m)');
title('Disk Position (m)');

%axis square
axis equal

% Remove temporary variables
clear temp_bxyz


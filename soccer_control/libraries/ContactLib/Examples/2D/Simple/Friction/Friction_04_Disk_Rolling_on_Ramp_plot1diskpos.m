% Code to plot simulation results from Friction_04_Disk_Rolling_on_Ramp
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Friction_04_Disk_Rolling_on_Ramp)
catch
    h1_Friction_04_Disk_Rolling_on_Ramp=figure('Name','Friction_04_Disk_Rolling_on_Ramp');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Friction_04_Disk_Rolling_on_Ramp','var'))
    sim('Friction_04_Disk_Rolling_on_Ramp')
end

% Get simulation results
temp_bxyz = logsout_Friction_04_Disk_Rolling_on_Ramp.get('Disk_xyz');
temp_trq = logsout_Friction_04_Disk_Rolling_on_Ramp.get('Input_Torque');
simlog_t = simlog_Friction_04_Disk_Rolling_on_Ramp.Planar_Joint.Rz.w.series.time;
simlog_wDisk = simlog_Friction_04_Disk_Rolling_on_Ramp.Planar_Joint.Rz.w.series.values('rev/s');

temp_wksp = get_param('Friction_04_Disk_Rolling_on_Ramp','ModelWorkspace');
temp_floor = getVariable(temp_wksp,'Floor');

temp_colorOrder = get(gca,'DefaultAxesColorOrder');

% Plot results
simlog_handles(1) = subplot(2, 1, 1);
cla
patch(sin(0:0.1:2*pi)*0.2+temp_bxyz.Values.Data(1,1),cos(0:0.1:2*pi)*0.2+temp_bxyz.Values.Data(1,3),temp_colorOrder(1,:),'FaceAlpha',0.2);
hold on
plot(temp_bxyz.Values.Data(:,1),temp_bxyz.Values.Data(:,3),'LineWidth',1);
patch(sin(0:0.1:2*pi)*0.2+temp_bxyz.Values.Data(end,1),cos(0:0.1:2*pi)*0.2+temp_bxyz.Values.Data(end,3),temp_colorOrder(1,:))
plot([-1 1]*temp_floor.length*0.5*cosd(temp_floor.angle),...
    [1 -1]*temp_floor.length*0.5*sind(-10),...
    'Color',[1 1 1]*0.5,'LineWidth',3);
hold off
xlabel('Horizontal Position (m)');
ylabel('Vertical Position (m)');
title('Disk Position (m)');
axis equal

simlog_handles(2) = subplot(2, 1, 2);
yyaxis left
plot(simlog_t,simlog_wDisk,'LineWidth',1);
ylabel('Speed (rev/s)');
yyaxis right
grid on
plot(temp_trq.Values.Time,temp_trq.Values.Data,'LineWidth',1);
xlabel('Time (s)');
ylabel('Torque (N*m)');
title('Input Torque');

% Remove temporary variables
clear temp_bxyz temp_wksp temp_trq temp_floor temp_colorOrder
clear simlog_t simlog_handles simlog_wDisk


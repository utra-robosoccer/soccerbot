% Code to plot simulation results from Gripper_2Belts
%% Plot Description:
%
% The plot below shows the 3D trajectory of the box moved by the gripper.
%
% Copyright 2017-2019 The MathWorks, Inc.

% Generate simulation results if they don't exist
if ~exist('simlog_Gripper_2Belts', 'var')
    sim('Gripper_2Belts')
end

% Reuse figure if it exists, else create new figure
if ~exist('h1_Gripper_2Belts', 'var') || ...
        ~isgraphics(h1_Gripper_2Belts, 'figure')
    h1_Gripper_2Belts = figure('Name', 'Gripper_2Belts');
end
figure(h1_Gripper_2Belts)
clf(h1_Gripper_2Belts)

temp_colororder = get(gca,'defaultAxesColorOrder');

% Get simulation results
simlog_t = simlog_Gripper_2Belts.Init_Box_6_DOF_Joint.Px.p.series.time;
simlog_boxPx = simlog_Gripper_2Belts.Init_Box_6_DOF_Joint.Px.p.series.values('m');
simlog_boxPy = simlog_Gripper_2Belts.Init_Box_6_DOF_Joint.Py.p.series.values('m');
simlog_boxPz = simlog_Gripper_2Belts.Init_Box_6_DOF_Joint.Pz.p.series.values('m');

% Plot results
plot3(simlog_boxPx, simlog_boxPy, simlog_boxPz, 'LineWidth', 1)
view(-159,16);
set(gca,'ZLim',[0 0.7]);
axis equal
set(gca,...
    'XTickLabel','','ZTickLabel','','YTickLabel','')
grid on
box on
set(gca,'XLim',[-0.7 0.3],'ZLim',[0 0.7],'TickLength',[0 0]);
title('Box Trajectory')

% Remove temporary variables
clear simlog_t simlog_boxPx simlog_boxPy simlog_boxPz
clear temp_colororder


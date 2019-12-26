% Code to plot simulation results from sm_robot_6legs_4bar
%% Plot Description:
%
% The plot below shows the cam angles and torque required to move the robot.
%
% Copyright 2018-2019 The MathWorks, Inc.

% Generate simulation results if they don't exist
if ~exist('simlog_sm_robot_6legs_4bar', 'var')
    sim('sm_robot_6legs_4bar')
end

% Reuse figure if it exists, else create new figure
if ~exist('h1_sm_robot_6legs_4bar', 'var') || ...
        ~isgraphics(h1_sm_robot_6legs_4bar, 'figure')
    h1_sm_robot_6legs_4bar = figure('Name', 'sm_robot_6legs_4bar');
end
figure(h1_sm_robot_6legs_4bar)
clf(h1_sm_robot_6legs_4bar)

temp_colororder = get(gca,'defaultAxesColorOrder');

% Get simulation results
simlog_t = simlog_sm_robot_6legs_4bar.Robot.Linkage_1.Revolute_Cam_L12.Rz.q.series.time;
simlog_qL = simlog_sm_robot_6legs_4bar.Robot.Linkage_1.Revolute_Cam_L12.Rz.q.series.values('rev');
simlog_qR = simlog_sm_robot_6legs_4bar.Robot.Linkage_2.Revolute_Cam_L12.Rz.q.series.values('rev');

simlog_trqLR = logsout_sm_robot_6legs_4bar.get('trq_LR');

% Plot results
simlog_handles(1) = subplot(2, 1, 1);
plot(simlog_t, simlog_qL, simlog_t, simlog_qR, 'LineWidth', 1)
hold off
grid on
title('Cam Angle')
ylabel('Angle (rev)')
legend({'Left','Right'},'Location','Best');

simlog_handles(2) = subplot(2, 1, 2);
plot(simlog_trqLR.Values.Time, simlog_trqLR.Values.Data, 'LineWidth', 1)
grid on
title('Actuator Torque')
ylabel('Torque (Nm)')
xlabel('Time (s)')

linkaxes(simlog_handles, 'x')

% Remove temporary variables
clear simlog_t simlog_handles temp_colororder
clear simlog_qL simlog_qR simlog_trqLR 


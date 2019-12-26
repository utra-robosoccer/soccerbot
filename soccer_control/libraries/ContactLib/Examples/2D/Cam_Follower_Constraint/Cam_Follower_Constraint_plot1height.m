% Code to plot simulation results from Cam_Follower_Constraint
%% Plot Description:
%
% The plot below shows the height of the follower and the torque applied at
% the cam.
%
% Copyright 2017-2019 The MathWorks, Inc.

% Generate simulation results if they don't exist
if ~exist('simlog_Cam_Follower_Constraint', 'var')
    sim('Cam_Follower_Constraint')
end

% Reuse figure if it exists, else create new figure
if ~exist('h1_Cam_Follower_Constraint', 'var') || ...
        ~isgraphics(h1_Cam_Follower_Constraint, 'figure')
    h1_Cam_Follower_Constraint = figure('Name', 'Cam_Follower_Constraint');
end
figure(h1_Cam_Follower_Constraint)
clf(h1_Cam_Follower_Constraint)

temp_colororder = get(gca,'defaultAxesColorOrder');

% Get simulation results
simlog_t = simlog_Cam_Follower_Constraint.Prismatic_Follower.Pz.p.series.time;
simlog_zF     = simlog_Cam_Follower_Constraint.Prismatic_Follower.Pz.p.series.values('m');
simlog_trqCam = logsout_Cam_Follower_Constraint.get('actuator_torque_cam');

% Plot results
simlog_handles(1) = subplot(2, 1, 1);
plot(simlog_t, simlog_zF, 'LineWidth', 1)
hold off
grid on
title('Follower Height')
ylabel('Height (m)')

simlog_handles(2) = subplot(2, 1, 2);
plot(simlog_trqCam.Values.Time, simlog_trqCam.Values.Data, 'LineWidth', 1)
grid on
title('Cam Actuator Torque')
ylabel('Torque (N*m)')
xlabel('Time (s)')

linkaxes(simlog_handles, 'x')

% Remove temporary variables
clear simlog_t simlog_handles temp_colororder
clear simlog_zF simlog_trqCam 


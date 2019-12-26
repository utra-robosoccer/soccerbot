% Code to plot simulation results from sm_tread_drive
%% Plot Description:
%
% The plot below shows the speed of the tread drive.
%
% Copyright 2017-2019 The MathWorks, Inc.

% Generate simulation results if they don't exist
if ~exist('simlog_sm_tread_drive', 'var')
    sim('sm_tread_drive')
end

% Reuse figure if it exists, else create new figure
if ~exist('h1_sm_tread_drive', 'var') || ...
        ~isgraphics(h1_sm_tread_drive, 'figure')
    h1_sm_tread_drive = figure('Name', 'sm_tread_drive');
end
figure(h1_sm_tread_drive)
clf(h1_sm_tread_drive)

%temp_colororder = get(gca,'defaultAxesColorOrder');

% Get simulation results
simlog_t = simlog_sm_tread_drive.Bushing_Tread.Px.p.series.time;
simlog_treadPxv = simlog_sm_tread_drive.Bushing_Tread.Px.v.series.values('m/s');
simlog_treadPyv = simlog_sm_tread_drive.Bushing_Tread.Py.v.series.values('m/s');

simlog_treadPx = simlog_sm_tread_drive.Bushing_Tread.Px.p.series.values('m');
simlog_treadPy = simlog_sm_tread_drive.Bushing_Tread.Py.p.series.values('m');

simlog_treadwr = logsout_sm_tread_drive.get('w_treadr');


% Plot results
simlog_handles(1) = subplot(2, 1, 1);
plot(simlog_t, sqrt(simlog_treadPxv.^2.+simlog_treadPyv.^2).*sign(simlog_treadwr.Values.Data), 'LineWidth', 1)
grid on
box on
title('Tread Speed')
xlabel('Time (s)');
ylabel('Speed (m/s)');

simlog_handles(2) = subplot(2, 1, 2);
plot(simlog_treadPx, simlog_treadPy, 'LineWidth', 1)
axis equal
grid on
box on
title('Tread Position')
xlabel('Position x (m)');
ylabel('Position y (m)');

% Remove temporary variables
clear simlog_t temp_colororder simlog_handles
clear simlog_treadPxv simlog_treadPyv simlog_treadPx simlog_treadPy
clear simlog_treadwr


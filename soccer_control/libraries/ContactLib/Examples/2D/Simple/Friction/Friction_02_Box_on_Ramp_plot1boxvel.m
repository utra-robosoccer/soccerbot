% Code to plot simulation results from Friction_02_Box_on_Ramp
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Friction_02_Box_on_Ramp)
catch
    h1_Friction_02_Box_on_Ramp=figure('Name','Friction_02_Box_on_Ramp');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Friction_02_Box_on_Ramp','var'))
    sim('Friction_02_Box_on_Ramp')
end

% Get simulation results
simlog_t = simlog_Friction_02_Box_on_Ramp.Revolute_Joint.Rz.q.series.time;
simlog_qRamp = simlog_Friction_02_Box_on_Ramp.Revolute_Joint.Rz.q.series.values('deg');
simlog_vBox = simlog_Friction_02_Box_on_Ramp.Planar_Joint.Py.v.series.values('m/s');

% Plot results
simlog_handles(1) = subplot(2, 1, 1);
plot(simlog_t, simlog_qRamp, 'LineWidth', 1)
grid on
ylabel('Angle (deg)');
title('Box on Ramp, Contact Forces: Ramp Angle');

simlog_handles(2) = subplot(2, 1, 2);
plot(simlog_t, simlog_vBox, 'LineWidth', 1)
grid on
ylabel('Speed (m/s)');
title('Box on Ramp, Contact Forces: Box Velocity');
xlabel('Time (s)');

linkaxes(simlog_handles, 'x')

% Remove temporary variables
clear simlog_t simlog_qRamp simlog_vBox simlog_handles


% Code to plot simulation results from Friction_03_Double_Pendulum_Constraint
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Friction_03_Double_Pendulum_Constraint)
catch
    h1_Friction_03_Double_Pendulum_Constraint=figure('Name','Friction_03_Double_Pendulum_Constraint');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Friction_03_Double_Pendulum_Constraint','var'))
    sim('Friction_03_Double_Pendulum_Constraint')
end

% Get simulation results
simlog_t = simlog_Friction_03_Double_Pendulum_Constraint.Revolute_BaseArm1.Rz.q.series.time;
simlog_qArm1 = simlog_Friction_03_Double_Pendulum_Constraint.Revolute_BaseArm1.Rz.q.series.values('deg');
simlog_wArm1Arm2 = simlog_Friction_03_Double_Pendulum_Constraint.Revolute_Arm1Arm2.Rz.w.series.values('deg/s');

% Plot results
simlog_handles(1) = subplot(2, 1, 1);
plot(simlog_t, simlog_qArm1, 'LineWidth', 1)
grid on
ylabel('Angle (deg)');
title('Angle of Arm 1');

simlog_handles(2) = subplot(2, 1, 2);
plot(simlog_t, simlog_wArm1Arm2, 'LineWidth', 1)
grid on
ylabel('Speed (deg/s)');
title('Rotational Speed of Arm1-Arm2 Joint');
xlabel('Time (s)');

linkaxes(simlog_handles, 'x')

% Remove temporary variables
clear simlog_t simlog_qArm1 simlog_wArm1Arm2 simlog_handles


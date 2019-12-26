% Code to plot simulation results from Collision_03_Disk_Finite_Plane_Spin
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h2_Collision_03_Disk_Finite_Plane_Spin)
catch
    h2_Collision_03_Disk_Finite_Plane_Spin=figure('Name','Collision_03_Disk_Finite_Plane_Spin');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Collision_03_Disk_Finite_Plane_Spin','var'))
    sim('Collision_03_Disk_Finite_Plane_Spin')
end

% Get simulation results
simlog_fn = logsout_Collision_03_Disk_Finite_Plane_Spin.get('Contact_Force');

% Plot results
plot(simlog_fn.Values.Time,simlog_fn.Values.Data,'LineWidth',1);
grid on
ylabel('Force (N)');
title('Contact Force (m)');
xlabel('Time (s)');

% Remove temporary variables
clear simlog_fn


% Code to plot simulation results from Collision_02_Disk_Finite_Plane_Fixed
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h2_Collision_02_Disk_Finite_Plane_Fixed)
catch
    h2_Collision_02_Disk_Finite_Plane_Fixed=figure('Name','Collision_02_Disk_Finite_Plane_Fixed');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Collision_02_Disk_Finite_Plane_Fixed','var'))
    sim('Collision_02_Disk_Finite_Plane_Fixed')
end

% Get simulation results
simlog_fn = logsout_Collision_02_Disk_Finite_Plane_Fixed.get('Contact_Force');

% Plot results
plot(simlog_fn.Values.Time,simlog_fn.Values.Data,'LineWidth',1);
grid on
ylabel('Force (N)');
title('Contact Force (m)');
xlabel('Time (s)');

% Remove temporary variables
clear simlog_fn


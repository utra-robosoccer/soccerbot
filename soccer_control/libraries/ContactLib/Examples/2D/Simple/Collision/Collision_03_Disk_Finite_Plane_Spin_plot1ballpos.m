% Code to plot simulation results from Collision_03_Disk_Finite_Plane_Spin
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Collision_03_Disk_Finite_Plane_Spin)
catch
    h1_Collision_03_Disk_Finite_Plane_Spin=figure('Name','Collision_03_Disk_Finite_Plane_Spin');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Collision_03_Disk_Finite_Plane_Spin','var'))
    sim('Collision_03_Disk_Finite_Plane_Spin')
end

% Get simulation results
temp_bxyz = logsout_Collision_03_Disk_Finite_Plane_Spin.get('Ball_xyz');

% Plot results
plot(temp_bxyz.Values.Data(:,1),temp_bxyz.Values.Data(:,3),'LineWidth',1);
grid on
xlabel('Horizontal Position (m)');
ylabel('Vertical Position (m)');
title('Ball Position (m)');

% Remove temporary variables
clear temp_bxyz


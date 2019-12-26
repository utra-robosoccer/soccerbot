% Code to plot simulation results from Collision_02_Disk_Finite_Plane_Fixed
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Collision_02_Disk_Finite_Plane_Fixed)
catch
    h1_Collision_02_Disk_Finite_Plane_Fixed=figure('Name','Collision_02_Disk_Finite_Plane_Fixed');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Collision_02_Disk_Finite_Plane_Fixed','var'))
    sim('Collision_02_Disk_Finite_Plane_Fixed')
end

% Get simulation results
temp_bpx = simlog_Collision_02_Disk_Finite_Plane_Fixed.Planar_Joint.Px.p.series;
temp_bpy = simlog_Collision_02_Disk_Finite_Plane_Fixed.Planar_Joint.Py.p.series;

% Plot results
plot(temp_bpy.values,temp_bpx.values,'LineWidth',1);
grid on
xlabel('Horizontal Position (m)');
ylabel('Vertical Position (m)');
title('Ball Position (m)');

% Remove temporary variables
clear temp_bpx temp_bpy


% Code to plot simulation results from Collision_01_Ball_Infinite_Plane
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Collision_01_Ball_Infinite_Plane)
catch
    h1_Collision_01_Ball_Infinite_Plane=figure('Name','Collision_01_Ball_Infinite_Plane');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Collision_01_Ball_Infinite_Plane','var'))
    sim('Collision_01_Ball_Infinite_Plane')
end

% Get simulation results
temp_bpx = simlog_Collision_01_Ball_Infinite_Plane.Planar_Joint.Px.p.series.values;
temp_bpy = simlog_Collision_01_Ball_Infinite_Plane.Planar_Joint.Py.p.series.values;

% Plot results
plot(temp_bpy,temp_bpx,'LineWidth',1);
grid on
xlabel('Horizontal Position (m)');
ylabel('Vertical Position (m)');
title('Ball Position (m)');

% Remove temporary variables
clear temp_bpx temp_bpy


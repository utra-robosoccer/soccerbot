% Code to plot simulation results from Collision_01_Ball_Infinite_Plane
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h2_Collision_01_Ball_Infinite_Plane)
catch
    h2_Collision_01_Ball_Infinite_Plane=figure('Name','Collision_01_Ball_Infinite_Plane');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Collision_01_Ball_Infinite_Plane','var'))
    sim('Collision_01_Ball_Infinite_Plane')
end

% Get simulation results
simlog_t = simlog_Collision_01_Ball_Infinite_Plane.Translational_Simscape_Interface.Ideal_Force_Sensor.F.series.time;
simlog_fn = simlog_Collision_01_Ball_Infinite_Plane.Translational_Simscape_Interface.Ideal_Force_Sensor.F.series.values('N');

% Plot results
plot(simlog_t,simlog_fn,'LineWidth',1);
grid on
ylabel('Force (N)');
title('Contact Force (m)');
xlabel('Time (s)');

% Remove temporary variables
clear simlog_t simlog_fn


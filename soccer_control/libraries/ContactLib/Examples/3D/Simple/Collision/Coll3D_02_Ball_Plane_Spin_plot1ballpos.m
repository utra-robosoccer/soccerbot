% Code to plot simulation results from Coll3D_02_Ball_Plane_Spin
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Coll3D_02_Ball_Plane_Spin)
catch
    h1_Coll3D_02_Ball_Plane_Spin=figure('Name','Coll3D_02_Ball_Plane_Spin');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Coll3D_02_Ball_Plane_Spin','var'))
    sim('Coll3D_02_Ball_Plane_Spin')
end

% Get simulation results
temp_bpx = simlog_Coll3D_02_Ball_Plane_Spin.x6_DOF_Joint.Px.p.series;
temp_bpy = simlog_Coll3D_02_Ball_Plane_Spin.x6_DOF_Joint.Py.p.series;
temp_bpz = simlog_Coll3D_02_Ball_Plane_Spin.x6_DOF_Joint.Pz.p.series;

% Plot results
subplot(2,1,1);
plot(temp_bpz.time,temp_bpz.values,'LineWidth',1);
ylim([-1 max(temp_bpz.values)*1.5]);xlim([0 max(tout)]);
grid on
ylabel('Ball Height (m)');
title('Ball Height (m)');
xlabel('Time (s)');

subplot(2,1,2);
cla;
patch([-5 5 5 -5],[-3 -3 3 3],[0.7 0.7 0.7]);
hold on
plot(temp_bpy.values,-temp_bpx.values,'LineWidth',1);
hold off
box on
axis([-8 8 -5 5])
title('Ball Position on Plane');
ylabel('X-Position (m)');
xlabel('Y-Position (m)');

% Remove temporary variables
clear temp_bpx temp_bpy temp_bpz


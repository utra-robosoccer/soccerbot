% Code to plot simulation results from Frict3D_01_Box_on_Table
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Frict3D_01_Box_on_Table)
catch
    h1_Frict3D_01_Box_on_Table=figure('Name','Frict3D_01_Box_on_Table');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Frict3D_01_Box_on_Table','var'))
    sim('Frict3D_01_Box_on_Table')
end

% Get simulation results
temp_bpx = simlog_Frict3D_01_Box_on_Table.x6_DOF_Joint.Px.p.series;
temp_bpy = simlog_Frict3D_01_Box_on_Table.x6_DOF_Joint.Py.p.series;

% Plot results
fill([-1 1 1 -1]*1.5,[-1 -1 1 1]*1.5,[0.7 0.7 0.7]);
hold on
plot(temp_bpx.values,temp_bpy.values,'LineWidth',2,'Color',[0 0.6 1]);
box on
hold off
axis([-2 2 -2 2]);
axis equal
title('Box Position on Table');
ylabel('Y-Position (m)');
xlabel('X-Position (m)');

% Remove temporary variables
clear temp_bpx temp_bpy


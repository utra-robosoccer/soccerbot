% Code to plot simulation results from Coll3D_03_Balls_in_Box
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Coll3D_03_Balls_in_Box)
catch
    h1_Coll3D_03_Balls_in_Box=figure('Name','Coll3D_03_Balls_in_Box');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Coll3D_03_Balls_in_Box','var'))
    sim('Coll3D_03_Balls_in_Box')
end

% Get simulation results
temp_b1px = simlog_Coll3D_03_Balls_in_Box.x6_DOF_Joint_Ball_1.Px.p.series;
temp_b1py = simlog_Coll3D_03_Balls_in_Box.x6_DOF_Joint_Ball_1.Py.p.series;
temp_b1pz = simlog_Coll3D_03_Balls_in_Box.x6_DOF_Joint_Ball_1.Pz.p.series;
temp_b2px = simlog_Coll3D_03_Balls_in_Box.x6_DOF_Joint_Ball_2.Px.p.series;
temp_b2py = simlog_Coll3D_03_Balls_in_Box.x6_DOF_Joint_Ball_2.Py.p.series;
temp_b2pz = simlog_Coll3D_03_Balls_in_Box.x6_DOF_Joint_Ball_2.Pz.p.series;
temp_b3px = simlog_Coll3D_03_Balls_in_Box.x6_DOF_Joint_Ball_3.Px.p.series;
temp_b3py = simlog_Coll3D_03_Balls_in_Box.x6_DOF_Joint_Ball_3.Py.p.series;
temp_b3pz = simlog_Coll3D_03_Balls_in_Box.x6_DOF_Joint_Ball_3.Pz.p.series;

% Plot results
plot3(temp_b1px.values,temp_b1py.values,temp_b1pz.values,'r');
hold on
plot3(temp_b2px.values,temp_b2py.values,temp_b2pz.values,'b');
plot3(temp_b3px.values,temp_b3py.values,temp_b3pz.values,'Color',[168 164 0]/255);
hold off
grid on
box on
axis equal
xlabel('Length (m)');
ylabel('Width (m)');
zlabel('Height (m)');
title('Ball Position in Box (m)');

% Remove temporary variables
clear temp_b1px temp_b1py temp_b1pz 
clear temp_b2px temp_b2py temp_b2pz
clear temp_b3px temp_b3py temp_b3pz


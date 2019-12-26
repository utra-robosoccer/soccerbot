% Code to plot simulation results from Frict3D_04_Ball_on_Ball
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Frict3D_04_Ball_on_Ball)
catch
    h1_Frict3D_04_Ball_on_Ball=figure('Name','Frict3D_04_Ball_on_Ball');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Frict3D_04_Ball_on_Ball','var'))
    sim('Frict3D_04_Ball_on_Ball')
end

% Get simulation results
temp_b1rx = simlog_Frict3D_04_Ball_on_Ball.Gimbal_Joint.Rx.q.series;
temp_b1ry = simlog_Frict3D_04_Ball_on_Ball.Gimbal_Joint.Ry.q.series;
temp_b1rz = simlog_Frict3D_04_Ball_on_Ball.Gimbal_Joint.Rz.q.series;

temp_b2pz = simlog_Frict3D_04_Ball_on_Ball.Telescoping_Joint.Pz.p.series;
temp_b2Q = simlog_Frict3D_04_Ball_on_Ball.Telescoping_Joint.S.Q.series.values;

temp_rotang = (acos(temp_b2Q(1:4:end)))*2*180/pi;

% Plot results
ah(1) = subplot(2,1,1);
plot(temp_b1rx.time,temp_b1rx.values,'r','LineWidth',1)
hold on
plot(temp_b1ry.time,temp_b1ry.values,'g','LineWidth',1,'LineStyle','--')
plot(temp_b1rz.time,temp_b1rz.values,'b','LineWidth',1,'LineStyle','-.')
hold off
grid on
title('Angles, Ball 1');
ylabel('Angle (deg)');
legend({'Rx','Ry','Rz'},'Location','Best')

ah(2) = subplot(2,1,2);
plot(temp_b2pz.time,temp_rotang,'k','LineWidth',1)
grid on
title('Rotation Angle, Ball 2');
ylabel('Angle (deg)');
legend({'abs(Rotation Angle)'},'Location','Best')

linkaxes(ah,'x');

% Remove temporary variables
clear temp_b1rx temp_b2rx temp_b3rx temp_b2pz temp_b2Q temp_rotang

% Code to plot simulation results from Frict3D_03_Board_on_Balls
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Frict3D_03_Board_on_Balls)
catch
    h1_Frict3D_03_Board_on_Balls=figure('Name','Frict3D_03_Board_on_Balls');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Frict3D_03_Board_on_Balls','var'))
    sim('Frict3D_03_Board_on_Balls')
end

% Get simulation results
temp_barx = simlog_Frict3D_03_Board_on_Balls.Gimbal_Joint_Ball_1.Rx.q.series;
temp_bary = simlog_Frict3D_03_Board_on_Balls.Gimbal_Joint_Ball_1.Ry.q.series;
temp_barz = simlog_Frict3D_03_Board_on_Balls.Gimbal_Joint_Ball_1.Rz.q.series;

temp_bopz = simlog_Frict3D_03_Board_on_Balls.Bushing_Joint.Pz.p.series;
temp_bopx = simlog_Frict3D_03_Board_on_Balls.Bushing_Joint.Px.p.series;
temp_bopy = simlog_Frict3D_03_Board_on_Balls.Bushing_Joint.Py.p.series;
temp_borz = simlog_Frict3D_03_Board_on_Balls.Bushing_Joint.Rz.q.series;


% Plot results
ah(1) = subplot(2,1,1);
plot(temp_barx.time,temp_barx.values,'LineWidth',1)
hold on
plot(temp_bary.time,temp_bary.values,'LineWidth',1,'LineStyle','--')
plot(temp_barz.time,temp_barz.values,'LineWidth',1)
hold off
grid on
title('Ball Angles');
ylabel('Angle (deg)');
legend({'Rx','Ry','Rz'},'Location','Best')

ah(2) = subplot(2,1,2);
plot(temp_bopx.time,temp_bopx.values,'LineWidth',1)
hold on
plot(temp_bopy.time,temp_bopy.values,'LineWidth',1)
plot(temp_bopz.time,temp_bopz.values,'LineWidth',1)
plot(temp_borz.time,temp_borz.values*pi/180,'LineWidth',1)
hold off
grid on
title('Table Position');
ylabel('Pos (m), Angle (rad)');
legend({'Px','Py','Pz','Rz (rad)'},'Location','Best')

linkaxes(ah,'x');

% Remove temporary variables
clear temp_barx temp_bary temp_barz temp_bopx temp_bopy temp_bopz temp_borz

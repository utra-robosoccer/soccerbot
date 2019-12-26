% Code to plot simulation results from Frict3D_06_Ball_on_Balls
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Frict3D_06_Ball_on_Balls)
catch
    h1_Frict3D_06_Ball_on_Balls=figure('Name','Frict3D_06_Ball_on_Balls');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Frict3D_06_Ball_on_Balls','var'))
    sim('Frict3D_06_Ball_on_Balls')
end

% Get simulation results
temp_b1rx = simlog_Frict3D_06_Ball_on_Balls.Gimbal_Joint_Ball_1.Rx.q.series;
temp_b1ry = simlog_Frict3D_06_Ball_on_Balls.Gimbal_Joint_Ball_1.Ry.q.series;
temp_b1rz = simlog_Frict3D_06_Ball_on_Balls.Gimbal_Joint_Ball_1.Rz.q.series;

temp_burx = simlog_Frict3D_06_Ball_on_Balls.Bushing_Joint.Rx.q.series;
temp_bury = simlog_Frict3D_06_Ball_on_Balls.Bushing_Joint.Ry.q.series;
temp_burz = simlog_Frict3D_06_Ball_on_Balls.Bushing_Joint.Rz.q.series;

% Plot results
ah(1) = subplot(2,1,1);
plot(temp_b1rx.time,temp_b1rx.values('deg'),'r','LineWidth',1)
hold on
plot(temp_b1ry.time,temp_b1ry.values('deg'),'g','LineWidth',1,'LineStyle','--')
plot(temp_b1rz.time,temp_b1rz.values('deg'),'b','LineWidth',1,'LineStyle','-.')
hold off
grid on
title('Ball 1 Orientation');
ylabel('Angle (deg)');
legend({'Rx','Ry','Rz'},'Location','Best')

ah(2) = subplot(2,1,2);
plot(temp_burx.time,temp_burx.values('deg'),'r','LineWidth',1)
hold on
plot(temp_bury.time,temp_bury.values('deg'),'g','LineWidth',1,'LineStyle','--')
plot(temp_burz.time,temp_burz.values('deg'),'b','LineWidth',1,'LineStyle','-.')
hold off
grid on
title('Upper Ball Orientation');
ylabel('Angle (deg)');
legend({'Rx','Ry','Rz'},'Location','Best')

linkaxes(ah,'x');

% Remove temporary variables
clear temp_b1rx temp_b1ry temp_b1rz temp_burx temp_bury temp_burz

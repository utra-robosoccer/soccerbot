% Code to plot simulation results from Geneva_Drive
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Geneva_Drive)
catch
    h1_Geneva_Drive=figure('Name','Geneva_Drive');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Geneva_Drive','var'))
    sim('Geneva_Drive')
end

% Get simulation results
simlog_t = simlog_Geneva_Drive.Revolute_Drive.Rz.q.series.time;
simlog_qDr = simlog_Geneva_Drive.Revolute_Drive.Rz.q.series.values('rev');
simlog_qGe = simlog_Geneva_Drive.Revolute_Geneva.Rz.q.series.values('rev');

% Plot results
temp_colorOrder = get(gca,'DefaultAxesColorOrder');
plot(simlog_t,simlog_qDr-simlog_qDr(1),'Color',[0 114 189]/255,'LineWidth',1);
hold on
plot(simlog_t,simlog_qGe-simlog_qGe(1),'Color',[222 125 0]/255,'LineWidth',1);

hold off
grid on
title('Geneva Drive Revolutions');
ylabel('Revolutions');
xlabel('Time (s)');
legend({'Drive Wheel','Geneva'},'Location','Best');

% Remove temporary variables
clear simlog_qDr simlog_qGe simlog_t



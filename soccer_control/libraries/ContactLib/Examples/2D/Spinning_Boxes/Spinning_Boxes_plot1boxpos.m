% Code to plot simulation results from Spinning_Boxes
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Spinning_Boxes)
catch
    h1_Spinning_Boxes=figure('Name','Spinning_Boxes');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Spinning_Boxes','var'))
    sim('Spinning_Boxes')
end

% Get simulation results
simlog_t = simlog_Spinning_Boxes.Prismatic_Joint.Pz.p.series.time;
simlog_hFlBox = simlog_Spinning_Boxes.Prismatic_Joint.Pz.p.series.values('m');

% Plot results
temp_colorOrder = get(gca,'DefaultAxesColorOrder');
plot(simlog_t,simlog_hFlBox,'Color',temp_colorOrder(2,:),'LineWidth',1);
grid on

title('Height of Floating Box');
ylabel('Height (m)');
xlabel('Time (s)');

% Remove temporary variables
clear simlog_t simlog_qDr simlog_qGe 
clear simlog_hFlBox




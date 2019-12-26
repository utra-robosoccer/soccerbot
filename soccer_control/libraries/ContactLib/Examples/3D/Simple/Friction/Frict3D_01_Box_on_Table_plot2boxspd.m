% Code to plot simulation results from Frict3D_01_Box_on_Table
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h2_Frict3D_01_Box_on_Table)
catch
    h2_Frict3D_01_Box_on_Table=figure('Name','Frict3D_01_Box_on_Table');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Frict3D_01_Box_on_Table','var'))
    sim('Frict3D_01_Box_on_Table')
end

% Get simulation results
temp_maxq = logsout_Frict3D_01_Box_on_Table.getElement('maxq').Values;
temp_boxspd = logsout_Frict3D_01_Box_on_Table.getElement('boxspd').Values;

% Plot results
ah(1) = subplot(2,1,1);
plot(temp_maxq.time,temp_maxq.Data,'LineWidth',1);
grid on
ylabel('Table Angle (deg)');
title('Table Angle');

ah(2) = subplot(2,1,2);
plot(temp_boxspd.time,temp_boxspd.Data,'LineWidth',1);
grid on
ylabel('Speed (m/s)');
xlabel('Time (s)');
title('Box Speed');

linkaxes(ah,'x');

% Remove temporary variables
clear temp_sprf temp_damf temp_nrmf


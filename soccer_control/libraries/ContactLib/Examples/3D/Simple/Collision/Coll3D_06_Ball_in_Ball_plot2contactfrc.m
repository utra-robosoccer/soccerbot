% Code to plot simulation results from Coll3D_06_Ball_in_Ball
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h2_Coll3D_06_Ball_in_Ball)
catch
    h2_Coll3D_06_Ball_in_Ball=figure('Name','Coll3D_06_Ball_in_Ball');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Coll3D_06_Ball_in_Ball','var'))
    sim('Coll3D_06_Ball_in_Ball')
end

% Get simulation results
temp_sprf = logsout_Coll3D_06_Ball_in_Ball.getElement('SphinSph_bus').Values.Spr_Force;
temp_damf = logsout_Coll3D_06_Ball_in_Ball.getElement('SphinSph_bus').Values.Damp_Force;
temp_nrmf = logsout_Coll3D_06_Ball_in_Ball.getElement('SphinSph_bus').Values.Normal_Force;

% Plot results
plot(temp_nrmf.time,temp_nrmf.Data,'LineWidth',1);
hold on
plot(temp_sprf.time,temp_sprf.Data,'LineWidth',1);
plot(temp_damf.time,temp_damf.Data,'LineWidth',1);
hold off
grid on
ylabel('Force (N)');
title('Contact Force (m)');
legend({'Normal Force','Spring Force','Damping Force'},'Location','Best');

% Remove temporary variables
clear temp_sprf temp_damf temp_nrmf


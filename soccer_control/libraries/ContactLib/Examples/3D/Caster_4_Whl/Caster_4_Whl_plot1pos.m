% Code to plot simulation results from Caster_4_Whl
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Caster_4_Whl)
catch
    h1_Caster_4_Whl=figure('Name','Caster_4_Whl');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Caster_4_Whl','var'))
    sim('Caster_4_Whl')
end

temp_colororder = get(gca,'defaultAxesColorOrder');

% Get simulation results
temp_cartxy = logsout_Caster_4_Whl.getElement('Cart xy').Values;
temp_cartx = -temp_cartxy.Data(:,2);
temp_carty = temp_cartxy.Data(:,1);

temp_hWksp = get_param(bdroot,'ModelWorkspace');
temp_table = temp_hWksp.getVariable('table');
temp_platform = temp_hWksp.getVariable('platform');
temp_axle = temp_hWksp.getVariable('axle');
temp_wheel = temp_hWksp.getVariable('whl');

%temp_xcir = sin(0:0.2:2*pi)*peg.rad;
%temp_ycir = cos(0:0.2:2*pi)*peg.rad;

% Plot board
patch([-1 1 1 -1]*temp_table.length/2,[-1 -1 1 1]*temp_table.length/2,[1 1 1]*0.8);
hold on

% Plot robot path
fill([-1 1 1 -1]*temp_platform.width/2+temp_cartx(1),...
     [-1 -1 1 1]*temp_platform.length/2+temp_carty(1),...
     [115 167 89]/200);
plot(temp_cartx,temp_carty,'Color',[115 167 89]/255,'LineWidth',3);

% Plot start and end points
pt1_h = plot(temp_cartx(1),temp_carty(1),'kd',...
    'MarkerFaceColor',temp_colororder(1,:),'MarkerSize',8);
pt2_h = plot(temp_cartx(end),temp_carty(end),'kd',...
    'MarkerFaceColor',temp_colororder(2,:),'MarkerSize',8);

hold off

box on
axis([[-1 1]*temp_table.length/2 [-1 1]*temp_table.length/2]*1.1);
axis equal
title('Cart Path');
ylabel('Position (m)');
xlabel('Position (m)');
legend([pt1_h pt2_h],{'Start','Finish'},'Location','Best');
% Remove temporary variables
clear temp_platform temp_cartxy temp_axle temp_table temp_hWksp temp_wheel


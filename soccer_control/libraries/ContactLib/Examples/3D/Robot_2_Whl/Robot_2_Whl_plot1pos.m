% Code to plot simulation results from Robot_2_Whl
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Robot_2_Whl)
catch
    h1_Robot_2_Whl=figure('Name','Robot_2_Whl');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Robot_2_Whl','var'))
    sim('Robot_2_Whl')
end

temp_colororder = get(gca,'defaultAxesColorOrder');

% Get simulation results
temp_robotxy = logsout_Robot_2_Whl.getElement('Robot xy').Values;
temp_robotx = -temp_robotxy.Data(:,2);
temp_roboty = temp_robotxy.Data(:,1);


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
fill(sin(0:0.1:2*pi)*temp_platform.radius+temp_robotx(1),...
     cos(0:0.1:2*pi)*temp_platform.radius+temp_roboty(1),...
     [115 167 89]/200);
plot(temp_robotx,temp_roboty,'Color',[115 167 89]/255,'LineWidth',3);
plot([1 1]*temp_robotx(1)+[1 -1]*temp_wheel.rad/2,...
     [1 1]*(temp_roboty(1)+temp_axle.length/2),'Color',[0.8 0.5 0.2],'LineWidth',2);
plot([1 1]*temp_robotx(1)+[1 -1]*temp_wheel.rad/2,...
     [1 1]*(temp_roboty(1)-temp_axle.length/2),'Color',[0.8 0.5 0.2],'LineWidth',2);

% Plot start and end points
pt1_h = plot(temp_robotx(1),temp_roboty(1),'kd',...
    'MarkerFaceColor',temp_colororder(1,:),'MarkerSize',8);
pt2_h = plot(temp_robotx(end),temp_roboty(end),'kd',...
    'MarkerFaceColor',temp_colororder(2,:),'MarkerSize',8);

hold off

box on
axis([[-1 1]*temp_table.length/2 [-1 1]*temp_table.length/2]*1.1);
axis equal
title('Robot Path');
ylabel('Position (m)');
xlabel('Position (m)');
legend([pt1_h pt2_h],{'Start','Finish'},'Location','Best');
% Remove temporary variables
clear temp_platform temp_robotxy temp_axle temp_table temp_hWksp temp_wheel


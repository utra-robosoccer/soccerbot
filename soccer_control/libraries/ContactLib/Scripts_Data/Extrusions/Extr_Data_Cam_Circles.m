function [xy_data] = Extr_Data_Cam_Circles(rad1, rad2, d, radh1, radh2, varargin)
%Extr_Data_Cam_Circles Produce extrusion data for a cam formed from two circles.
%   [xy_data] = Extr_Data_Cam_Circles(rad1, rad2, d, radh1, radh2)
%   This function returns x-y data for a cam formed from two circles
%   that are connected by lines tangent to the circles.
%
%   You can specify:
%       Radius, circle 1                       rad1
%       Radius, circle 2                       rad2
%       Distance between circle centers	d
%       Radius of hole at center of circle 1   radh1
%       Radius of hole at center of circle 2   radh2
%
%   To see a plot showing parameter values, enter the name
%   of the function with no arguments
%   >> Extr_Data_Cam_Circles
%
%   To see a plot created with your parameter values,
%   add 'plot' as the final argument
%   >> Extr_Data_Cam_Circles(4,3,5,1,0,'plot')

% Copyright 2012-2019 The MathWorks, Inc.

% Default data to show diagram
if (nargin == 0)
    rad1 = 4;
    rad2 = 3;
    d = 5;
    radh1 = 1;
    radh2 = 1; 
end

% Check if plot should be produced
if (isempty(varargin))
    showplot = 'n';
else
    showplot = varargin;
end

% Calculate angle for external tangent
theta1 = acos((rad1-rad2)/d)*180/pi;

% Start circle at 90 degrees to avoid self-intersecting polygons
theta_cir = [90:1:360+90]'*pi/180;
unit_cir = [cos(theta_cir), sin(theta_cir)];

% Calculate extrusion data
% Arc for rad1
theta_r1 = [(theta1:1:360-theta1)]'*pi/180;
unit_arc = [cos(theta_r1), sin(theta_r1)];
xy_data1 = [rad1 * unit_arc];
if(radh1>0)
    % Circle for radh1
    xy_data1 = [xy_data1(1,:);radh1 * flipud(unit_cir);xy_data1];
end

% Arc for rad2
theta_r2 = [(-theta1:1:theta1)]'*pi/180;
unit_arc = [cos(theta_r2), sin(theta_r2)];
xy_data2 = [rad2 * unit_arc]+repmat([d 0],size(unit_arc,1),1);
if(radh2>0)
    % Circle for radh2
    xy_data2 = [xy_data2;radh2 * flipud(unit_cir)+repmat([d 0],size(unit_cir,1),1);xy_data2(end,:)];
end

xy_data = [xy_data1;xy_data2];

% Plot diagram to show parameters and extrusion
if (nargin == 0 || strcmpi(showplot,'plot'))
    
    % Figure name
    figString = ['h1_' mfilename];
    % Only create a figure if no figure exists
    figExist = 0;
    fig_hExist = evalin('base',['exist(''' figString ''')']);
    if (fig_hExist)
        figExist = evalin('base',['ishandle(' figString ') && strcmp(get(' figString ', ''type''), ''figure'')']);
    end
    if ~figExist
        fig_h = figure('Name',figString);
        assignin('base',figString,fig_h);
    else
        fig_h = evalin('base',figString);
    end
    figure(fig_h)
    clf(fig_h)
    
    % Plot extrusion data
    patch(xy_data(:,1),xy_data(:,2),[1 1 1]*0.90,'EdgeColor','none');
    hold on
    plot(xy_data(:,1),xy_data(:,2),'-','Marker','o','MarkerSize',4,'LineWidth',2);
    axis('square');
    axis([-rad1 d+rad2 [-1 1]*(rad1+d+rad2)/2]*1.1);
    
    % Show parameters
    hold on
    plot([0 d],[0 0],'k-d','MarkerFaceColor','k','MarkerSize',10);
    text(d/2,0.1*max(rad1,rad2),'d');
    
    plot([d d+rad2*cos(theta1*pi/180)],[0 rad2*sin(theta1*pi/180)],'r-d','MarkerFaceColor','r');
    text(d+rad2*cos(theta1*pi/180)/2,rad2*sin(theta1*pi/180)/2,'{\color{red}rad2}');
    
    plot([0 rad1*cos(theta1*pi/180)],[0 rad1*sin(theta1*pi/180)],'g-d','MarkerFaceColor','g');
    text(rad1*cos(theta1*pi/180)/2,rad1*sin(theta1*pi/180)/2,'{\color{green}rad1}');
    
    title(['[xy\_data] = Extr\_Data\_Cam(rad1, rad2, d1);']);
    hold off
    box on
    clear xy_data
end

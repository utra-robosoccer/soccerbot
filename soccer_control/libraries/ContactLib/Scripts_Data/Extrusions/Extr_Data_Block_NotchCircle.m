function [xy_data] = Extr_Data_Block_NotchCircle(box_x, box_y, notch_rad, notch_th, box_r, varargin)
%Extr_Data_Block_NotchCircle Produce extrusion data for a rectangle with rounded corners.
%   [xy_data] = Extr_Data_Block_NotchCircle(height, width, radius, varargin)
%   This function returns x-y data for a rectangle with rounded corners.
%   You can specify:
%       Width                   box_x
%       Height                  box_y
%       Notch radius            notch_rad
%       Notch theta             notch_th
%       Radius, outer fillet    box_r
%
%   To see a plot showing parameter values, enter the name
%   of the function with no arguments
%   >> Extr_Data_Block_NotchCircle
%
%   To see a plot created with your parameter values,
%   add 'plot' as the final argument
%   >> Extr_Data_Block_NotchCircle(2,1,0.7,45,0.2,'plot')

% Copyright 2012-2019 The MathWorks, Inc.

% Default data to show diagram
if (nargin == 0)
    box_x = 0.02;
    box_y = 0.01;
    notch_rad = 0.007;
    notch_th = 45;
    box_r = 0.002;
end

if (nargin == 6)
    showplot = varargin{end};
else
    showplot = 'n';
end

% Check if plot should be produced
if (isempty(varargin))
    showplot = 'n';
end

%box_oy = b_D/2-(b_D+b_d)/4-notch_rad*cosd(notch_th);
%box_ox = b_b;

% Create outer profile
if (box_r>0)
    xyset1 = [box_x/2 0;box_x/2 box_y-box_r ];
    xyset3 = [box_x/2-box_r box_y;-box_x/2+box_r box_y];
    xyset5 = [-box_x/2 box_y-box_r;-box_x/2 0];
%    xyset7 = [box_ox/2 -box_oy/2+rad_o;box_ox/2 box_oy/2-rad_o];
    
    xyset2 = [Extr_Data_Ring(box_r,0,1,89)];
    xyset2(:,1) = xyset2(:,1)+(box_x/2-box_r);
    xyset2(:,2) = xyset2(:,2)+(box_y-box_r);
    xyset4 = [Extr_Data_Ring(box_r,0,91,179)];
    xyset4(:,1) = xyset4(:,1)+(-box_x/2+box_r);
    xyset4(:,2) = xyset4(:,2)+(box_y-box_r);
    xy_data = [xyset1; xyset2; xyset3; xyset4; xyset5];
else
    xyset1 = [box_x/2 box_y/2];
    xyset3 = [-box_x/2 box_y/2];
    xyset5 = [-box_x/2 -box_y/2];
    xy_data = [xyset1; xyset3; xyset5];
end

xyset6 = flipud(Extr_Data_Ring(notch_rad,-1,-notch_th+90,notch_th+90));
xyset6 = xyset6 + [0 -notch_rad*cosd(notch_th)];
xy_data=[xy_data;xyset6];

%xy_data = xy_data + [0 (b_D+b_d)/4+sph_r*cosd(race_th)];


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
    
    % Plot extrusion
    patch(xy_data(:,1),xy_data(:,2),[1 1 1]*0.90,'EdgeColor','none');
    hold on
    plot(xy_data(:,1),xy_data(:,2),'-','Marker','o','MarkerSize',4,'LineWidth',2);
    axis('equal');
    axis([-1.1 1.1 -0.6 1.6]*max(box_y/2,box_x/2));
    
    % Show parameters
    hold on
    
    plot([-1 -1]*0.4*box_x/2,[0 box_y],'r-d','MarkerFaceColor','r');
    text(-0.45*box_x/2,box_y*0.5,'{\color{red}box\_y}','HorizontalAlignment','right');
    
    plot([-box_x/2 box_x/2],[1 1]*box_y*0.2,'b-d','MarkerFaceColor','b');
    text(box_x/2*0.25,box_y*0.25,'{\color{blue}box\_ox}');
    
    radius_label_angle = 45*(pi/180);
    plot([box_x/2-box_r box_x/2-box_r*(1-cos(radius_label_angle))],[box_y-box_r box_y-box_r*(1-sin(radius_label_angle))],'k-d','MarkerFaceColor','k');
    text(box_x/2-2*box_r*(cos(radius_label_angle)),box_y-2*box_r*(sin(radius_label_angle)),'{\color{black}rad\_o}','HorizontalAlignment','center');

    plot([0 0],[0 notch_rad*1.2]-notch_rad*cosd(notch_th),'k:');
    plot([0 sind(notch_th)]*notch_rad,([0 cosd(notch_th)]*notch_rad)-notch_rad*cosd(notch_th),'k-d','MarkerFaceColor','k');
    text(notch_rad*sind(notch_th)*1.1,0,'{\color{black}notch\_rad}','HorizontalAlignment','left');
    
    arc1_r = 0.75*notch_rad;
    arc1 = linspace(0,notch_th,50)'*pi/180;
    plot(sin(arc1)*arc1_r,cos(arc1)*arc1_r-notch_rad*cosd(notch_th),'g--');
    plot(sin(arc1(1))*arc1_r,cos(arc1(1))*arc1_r-notch_rad*cosd(notch_th),'gd','MarkerFaceColor','g');
    plot(sin(arc1(end))*arc1_r,cos(arc1(end))*arc1_r-notch_rad*cosd(notch_th),'gd','MarkerFaceColor','g');
    text(-0.02*box_x,(0.75-cosd(notch_th))*notch_rad,'{\color{green}notch\_th}','HorizontalAlignment','right');
    
    title(['[xy\_data] = Extr\_Data\_Block\_NotchCircle(box\_x, box\_y, notch\_rad, notch\_th, box\_r)'],'FontSize',10);
    hold off
    box on
    clear xy_data
end

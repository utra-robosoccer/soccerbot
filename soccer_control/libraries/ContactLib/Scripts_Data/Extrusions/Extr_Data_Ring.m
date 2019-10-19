function [xy_data] = Extr_Data_Ring(ring_or, ring_ir, deg1, deg2, varargin)
%Extr_Data_Ring Produce extrusion data for a ring.
%   [xy_data] = Extr_Data_Ring(ring_or, ring_ir, deg1, deg2)
%   This function returns x-y data for a ring.
%   You can specify:
%       Outer radius	ring_or
%       Inner radius	ring_ir
%       Start angle     deg1
%       Finish angle	deg2
%
%   To see a plot showing parameter values, enter the name
%   of the function with no arguments
%   >> Extr_Data_Ring
%
%   To see a plot created with your parameter values,
%   add 'plot' as the final argument
%   >> Extr_Data_Ring(10,5,45,315,'plot')

% Copyright 2012-2017 The MathWorks, Inc.

% Default data to show diagram
if (nargin == 0)
    deg1 = 45;
    deg2 = 315;
    ring_or = 10;
    ring_ir = 5;
end

% Check if plot should be produced
if (isempty(varargin))
    showplot = 'n';
else
    showplot = varargin;
end

% Calculate extrusion data
theta = [(deg1:1:deg2) ]'*pi/180;
unit_circle = [cos(theta), sin(theta)];
if(ring_ir>0)
    xy_data = [ring_or * unit_circle; ring_ir * flipud(unit_circle)];
else
    xy_data = [ring_or * unit_circle];
end

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
    axis([-1.1 1.1 -1.1 1.1]*ring_or);
    
    % Show parameters
    hold on
    
    ir_label_ang = deg1+deg2*0.2;
    or_label_ang = deg1+deg2*0.5;
    plot([0 ring_or*(cos(or_label_ang*pi/180))],[0 ring_or*(sin(or_label_ang*pi/180))],'r-d','MarkerFaceColor','r');
    text(cos(or_label_ang*pi/180)*0.75*ring_or,sin(or_label_ang*pi/180)*0.75*ring_or,'{\color{red}ring\_or}');
    
    plot([0 ring_ir*(cos(ir_label_ang*pi/180))],[0 ring_ir*(sin(ir_label_ang*pi/180))],'g-d','MarkerFaceColor','g');
    text(cos(ir_label_ang*pi/180)*0.5*ring_ir,sin(ir_label_ang*pi/180)*0.5*ring_ir,'{\color{green}ring\_ir}');
    
    plot([0 ring_or],[0 0],'k:');
    plot([0 ring_ir*cos(deg1*pi/180)],[0 ring_ir*sin(deg1*pi/180)],'k:');
    plot([0 ring_ir*cos(deg2*pi/180)],[0 ring_ir*sin(deg2*pi/180)],'k:');
    
    arc1_r = 0.75*ring_ir;
    arc1 = linspace(0,deg1,50)'*pi/180;
    plot(cos(arc1)*arc1_r,sin(arc1)*arc1_r,'k-');
    plot(cos(arc1(1))*arc1_r,sin(arc1(1))*arc1_r,'kd','MarkerFaceColor','k');
    plot(cos(arc1(end))*arc1_r,sin(arc1(end))*arc1_r,'kd','MarkerFaceColor','k');
    text(cos(deg1/2*pi/180)*arc1_r*1.1,sin(deg1/2*pi/180)*arc1_r*1.1,'deg1');
    
    arc2_r = 0.25*ring_ir;
    arc2 = [(0:1:deg2)]'*pi/180;
    plot(cos(arc2)*arc2_r,sin(arc2)*arc2_r,'k-');
    plot(cos(arc2(1))*arc2_r,sin(arc2(1))*arc2_r,'kd','MarkerFaceColor','k');
    plot(cos(arc2(end))*arc2_r,sin(arc2(end))*arc2_r,'kd','MarkerFaceColor','k');
    text(cos(0.7*deg2*pi/180)*arc2_r*2,sin(0.7*deg2*pi/180)*arc2_r*2,'deg2');
    title(['[xy\_data] = Extr\_Data\_Ring(ring\_or, ring\_ir, deg1, deg2);']);
    hold off
    box on
    clear xy_data
end

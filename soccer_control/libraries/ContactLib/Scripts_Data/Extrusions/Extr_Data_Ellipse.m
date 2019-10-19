function [xy_data] = Extr_Data_Ellipse(a, b, deg1, deg2, ctr, varargin)
%Extr_Data_Ellipse Produce extrusion data for an ellipse.
%   [xy_data] = Extr_Data_Ellipse(a, b, deg1, deg2, ctr, varargin)
%   This function returns x-y data for a ring.
%   You can specify:
%       Radius 1        a
%       Radius 2        b
%       Start angle     deg1
%       Finish angle	deg2
%       Include Center	ctr
%
%   To see a plot showing parameter values, enter the name
%   of the function with no arguments
%   >> Extr_Data_Ellipse
%
%   To see a plot created with your parameter values,
%   add 'plot' as the final argument
%   >> Extr_Data_Ellipse(5,2,60,315,1,'plot')

% Copyright 2012-2017 The MathWorks, Inc.

% Default data to show diagram
if (nargin == 0)
    a = 5;
    b = 2;
    deg1 = 60;
    deg2 = 315;
    ctr = 1;
end

% Check if plot should be produced
if (isempty(varargin))
    showplot = 'n';
else
    showplot = varargin;
end

% Calculate quarter of ellipse
xdata = a:-a/50:0;
for i=1:length(xdata)
    ydata(i) = b/a*sqrt(a^2-xdata(i)^2);
end

% Full outer ellipse
xy_data = [xdata -fliplr(xdata(1:end-1)) -xdata(2:end) fliplr(xdata(1:end-1)); ydata fliplr(ydata(1:end-1)) -ydata(2:end) -fliplr(ydata(1:end-1))]';

% Determine start and stop
angles = unwrap(atan2(xy_data(:,2),xy_data(:,1)))*180/pi;
start = find(angles>=deg1);
start_ind = start(1);
finish = find(angles<=deg2);
finish_ind = finish(end);

% Determine connection between start and finish
if (ctr == 0)
    xy_data = [0 0; xy_data(start_ind:finish_ind,:)];
elseif (ctr == 1)
    xy_data = [xy_data(start_ind:finish_ind,:)];
else
    xy_data = [xy_data(start_ind:finish_ind,:);flipud(xy_data(start_ind:finish_ind,:))*ctr];
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
    
    %    plot(xy_data(:,1),xy_data(:,2),'-','Marker','.','MarkerSize',12,'LineWidth',1.5);
    axis('equal');
    axis([-1.1 1.1 -1.1 1.1]*max(a,b));
    
    % Show parameters
    hold on
    
    a_label_ang = 180;
    b_label_ang = 90;
    plot([0 a*(cos(a_label_ang*pi/180))],[0 a*(sin(a_label_ang*pi/180))],'r-d','MarkerFaceColor','r');
    text(cos(a_label_ang*pi/180)*0.75*a,sin(a_label_ang*pi/180)*0.75*a,'{\color{red}a}');
    
    plot([0 b*(cos(b_label_ang*pi/180))],[0 b*(sin(b_label_ang*pi/180))],'g-d','MarkerFaceColor','g');
    text(cos(b_label_ang*pi/180)*0.5*b,sin(b_label_ang*pi/180)*0.5*b,'{\color{green}b}');
    
    plot([0 a],[0 0],'k:');
    plot([0 b*cos(deg1*pi/180)],[0 b*sin(deg1*pi/180)],'k:');
    plot([0 b*cos(deg2*pi/180)],[0 b*sin(deg2*pi/180)],'k:');
    
    arc1_r = 0.6*b;
    arc1 = [(0:1:deg1)]'*pi/180;
    plot(cos(arc1)*arc1_r,sin(arc1)*arc1_r,'k-');
    plot(cos(arc1(1))*arc1_r,sin(arc1(1))*arc1_r,'kd','MarkerFaceColor','k');
    plot(cos(arc1(end))*arc1_r,sin(arc1(end))*arc1_r,'kd','MarkerFaceColor','k');
    text(cos(deg1/2*pi/180)*arc1_r*1.1,sin(deg1/2*pi/180)*arc1_r*1.1,'deg1');
    
    arc2_r = 0.25*b;
    arc2 = [(0:1:deg2)]'*pi/180;
    plot(cos(arc2)*arc2_r,sin(arc2)*arc2_r,'k-');
    plot(cos(arc2(1))*arc2_r,sin(arc2(1))*arc2_r,'kd','MarkerFaceColor','k');
    plot(cos(arc2(end))*arc2_r,sin(arc2(end))*arc2_r,'kd','MarkerFaceColor','k');
    text(cos(0.7*deg2*pi/180)*arc2_r*2,sin(0.7*deg2*pi/180)*arc2_r*2,'deg2');
    
    text(-a,0.75*a,'ctr = 0, center point included');
    text(-a,0.65*a,'ctr >0 & <1, hollow ellipse created');
    text(-a,0.55*a,'ctr = 1, chord connects ends');
    
    title(['[xy\_data] = Extr\_Data\_Ellipse(a, b, deg1, deg2, ctr);']);
    hold off
    box on
    clear xy_data
end

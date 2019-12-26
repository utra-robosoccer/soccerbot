function [xy_data] = Extr_Data_Link2Hole(L, W, r, varargin)
%Extr_Data_Link2Hole Produce extrusion data for a link with half a hole at each end.
%   [xy_data] = Extr_Data_Link2Hole(L, W, r)
%   This function returns x-y data for a link with half a hole at each end.
%   You can specify:
%       Length      L
%       Width       W
%       Hole radius r
%
%   To see a plot showing parameter values, enter the name
%   of the function with no arguments
%   >> Extr_Data_Link2Hole
%
%   To see a plot created with your parameter values,
%   add 'plot' as the final argument
%   >> Extr_Data_Link2Hole(10,5,2,'plot')

% Copyright 2012-2019 The MathWorks, Inc.

% Default data to show diagram
if (nargin == 0)
    L = 10;
    W = 5;
    r = 2;
end

% Check if plot should be produced
if (isempty(varargin))
    showplot = 'n';
else
    showplot = char(varargin);
end

top_xsec = [L/2 W/2; -L/2 W/2];
theta = (90:-1:-90)*pi/180;
left_semi_hole_xsec = [-L/2+r*cos(theta') r*sin(theta')];
bottom_xsec = [-L/2 -W/2; L/2 -W/2];
theta = (270:-1:90)*pi/180;
right_semi_hole_xsec = [L/2+r*cos(theta') r*sin(theta')];
xy_data = [ top_xsec; left_semi_hole_xsec; ...
    bottom_xsec; right_semi_hole_xsec];

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
    axis([-1 1 -1 1]*(L+W)*1.1/2);
    
    % Show parameters
    hold on
    
    plot([-L/2 L/2],[0 0],'r-d','MarkerFaceColor','r');
    text(-L/4,W/10,'{\color{red}L}');
    plot([L/4 L/4],[-W/2 W/2],'g-d','MarkerFaceColor','g');
    text(L/3.8,W/4,'{\color{green}W}');
    plot([-L/2 -L/2+r*sin(30*pi/180)],[0 r*cos(30*pi/180)],'k-d','MarkerFaceColor','k');
    text(-L/2+r*sin(30*pi/180)*1.4,1.4*r*cos(30*pi/180),'r');
    
    title(['[xy\_data] = Extr\_Data\_Link2Hole(L, W, r);']);
    
    hold off
    box on
    clear xy_data
end
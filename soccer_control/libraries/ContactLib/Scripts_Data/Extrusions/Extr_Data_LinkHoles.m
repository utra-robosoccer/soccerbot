function [xy_data] = Extr_Data_LinkHoles(L, W, r, num_holes, varargin)
%Extr_Data_LinkHoles Produce extrusion data for a link with an arbitrary
%number of holes and rounded ends.
%   [xy_data] = Extr_Data_LinkHoles(L, W, r, num_holes)
%   This function returns x-y data for a link with half a hole at one end.
%   You can specify:
%       Length      L
%       Width       W
%       Hole radius r
%       # holes 	num_holes
%
%   To see a plot showing parameter values, enter the name
%   of the function with no arguments
%   >> Extr_Data_LinkHoles
%
%   To see a plot created with your parameter values,
%   add 'plot' as the final argument
%   >> Extr_Data_LinkHoles(10,5,1,3,'plot')
%
% Copyright 2014-2019 The MathWorks, Inc.

% Default data to show diagram
if (nargin == 0)
    L = 15;
    W = 5;
    r = 1;
    num_holes = 4;
end

% Check if plot should be produced
if (isempty(varargin))
    showplot = 'n';
else
    showplot = char(varargin);
end


% Half length and width
hl = L / 2;
hw = W / 2;

% Rounded ends and outer boundary of member
theta = (-89:1:+90)' * pi/180; % CCW
rght_end = repmat([hl 0], 180, 1) + hw * [cos(theta), sin(theta)];
left_end = flipud(rght_end) * diag([-1 1]);
boundary = [rght_end; left_end];

% Circular holes
theta = (270:-1:-90)' * pi/180; % CW
hole = [[0 -hw]; r * [cos(theta), sin(theta)]; [0 -hw]];
holes = kron(linspace(-hl, +hl, num_holes)', repmat([1 0], 363, 1)) ...
    + repmat(hole, num_holes, 1);

xy_data = [boundary; holes];

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
    patch(xy_data(:,1),xy_data(:,2),[1 1 1]*0.95,'EdgeColor','none');
    hold on
    plot(xy_data(:,1),xy_data(:,2),'-','Marker','o','MarkerSize',4,'LineWidth',2);
    
    axis('equal');
    axis([-1 1 -1 1]*(L+W)*1.1/2);
    
    % Show parameters
    plot([-L/2 L/2],[0 0],'r-d','MarkerFaceColor','r');
    text(-L/4,W/10,'{\color{red}L}');
    plot([L/4 L/4],[-W/2 W/2],'g-d','MarkerFaceColor','g');
    text(L/3.8,W/4,'{\color{green}W}');
    plot([-L/2 -L/2+r*sin(30*pi/180)],[0 r*cos(30*pi/180)],'k-d','MarkerFaceColor','k');
    text(-L/2+r*sin(30*pi/180)*1.4,1.4*r*cos(30*pi/180),'r');
    
    title('[xy\_data] = Extr\_Data\_LinkHoles(L, W, r, num\_holes);');
    hold off
    box on
    clear xy_data
end



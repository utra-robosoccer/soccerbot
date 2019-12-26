function [xy_data] = Extr_Data_Box(box_ox, box_ix, varargin)
%Extr_Data_Box Produce extrusion data for a hollow box.
%   [xy_data] = Extr_Data_Box(box_ox, box_ix, <box_oy>, <box_iy>)
%   This function returns x-y data for a hollow box.
%   You can specify:
%       Box outer width (x)              box_ox
%       Box inner width (x)              box_ix
%       (optional) Box outer height (y)  box_oy
%       (optional) Box inner height (y)  box_iy
%
%   To see a plot showing parameter values, enter the name
%   of the function with no arguments
%   >> Extr_Data_Box
%
%   To see a plot created with your parameter values,
%   add 'plot' as the final argument
%   >> Extr_Data_Box(10,5,'plot')

% Copyright 2013-2019 The MathWorks, Inc.

% Default data to show diagram
if (nargin == 0)
    box_ox = 15;
    box_ix = 12;
    box_oy = 10;
    box_iy = 5;
elseif (nargin <= 3)
    box_oy = box_ox;
    box_iy = box_ix;
elseif (nargin > 3)
    box_oy = varargin{1};
    box_iy = varargin{2};
end

if (nargin==0)
    showplot = 'plot';
elseif (nargin==3 || nargin==5)
    showplot = varargin(end);
else
    showplot = 'n';
end

% Generate extrusion data
x_signs = [1 -1 -1 1]';
y_signs = [1 1 -1 -1]';
xy_data = [box_ox/2*x_signs box_oy/2*y_signs];
if(box_ix>0 && box_iy>0)
    xy_data =  [xy_data;box_ox/2 box_oy/2;box_ix/2*y_signs box_iy/2*x_signs;box_ix/2 box_iy/2];
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
    
    % Get max axis dimension
    maxd = max(box_ox,box_oy);
    
    % Plot extrusion
    patch(xy_data(:,1),xy_data(:,2),[1 1 1]*0.90,'EdgeColor','none');
    hold on
    plot(xy_data(:,1),xy_data(:,2),'-','Marker','o','MarkerSize',4,'LineWidth',2);
    axis('equal');
    axis([-1.1 1.1 -1.1 1.1]*maxd/2);
    
    % Show parameters
    hold on
    
    plot([-1 1]*box_ox/2,[0 0],'r-d','MarkerFaceColor','r');
    text(0.25*box_ox,0.025*maxd,'{\color{red}box\_ox}');
    
    plot([0 0],[-1 1]*box_oy/2,'g-d','MarkerFaceColor','g');
    text(0,-0.4*box_oy,'{\color{green}box\_oy}');
    
    if(box_ix>0 && box_iy>0)
        plot([-1 1]*box_ix/2,[1 1]*0.25*box_iy,'b-d','MarkerFaceColor','b');
        text(0.25*box_ix,0.03*maxd+0.25*box_iy,'{\color{blue}box\_ix}');
        
        plot([-1 -1]*0.25*box_ix,[-1 1]*box_iy/2,'k-d','MarkerFaceColor','k');
        text(-0.25*box_ix+0.03*maxd,0.4*box_iy,'{\color{black}box\_iy}');
    end
    title('[xy\_data] = Extr\_Data\_Box(box\_ox, box\_ix, <box\_oy, box\_iy>);');
    hold off
    box on
    clear xy_data
end

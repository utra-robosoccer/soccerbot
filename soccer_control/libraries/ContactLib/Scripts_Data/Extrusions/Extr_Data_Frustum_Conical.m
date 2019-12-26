function [xy_data] = Extr_Data_Frustum_Conical(cone_or, cone_th, varargin)
%Extr_Data_Frustum_Conical Produce *revolution* data for a conical frustum.
%   [xy_data] = Extr_Data_Frustum_Conical(cone_or, cone_th, <cone_h>, <cone_ir>)
%   This function returns x-y data for a conical frustrum.
%   You can specify:
%       Cone outer radius (m)             cone_or
%       Cone theta (deg)                  cone_th
%       (optional) Cone height (m)        cone_h
%       (optional) Cone inner radius (m)  cone_ir
%
%   To see a plot showing parameter values, enter the name
%   of the function with no arguments
%   >> Extr_Data_Frustum_Conical
%
%   To see a plot created with your parameter values,
%   add 'plot' as the final argument
%   >> Extr_Data_Frustum_Conical(5,60,4,4,'plot')

% Copyright 2017-2019 The MathWorks, Inc.

% Default data to show diagram
showplot = 'n';
if (nargin == 0)
    cone_or = 5;
    cone_th = 60;
    cone_h = 4;
    cone_ir = 4;
    showplot = 'plot';
elseif (nargin == 2)
    cone_h = cone_or*tand(cone_th);
    cone_ir = 0;
elseif (nargin == 3)
    cone_ir = 0;
    if ischar(varargin{1})   % PLOT REQUESTED
        showplot = varargin{1};
        cone_h = cone_or*tand(cone_th);
    else
        cone_h = varargin{1};    % Ad SPECIFIED
    end
elseif (nargin == 4)
    cone_h = varargin{1};    % Ad SPECIFIED
    if ischar(varargin{2})   % PLOT REQUESTED
        showplot = varargin{2};
        cone_ir = 0;
    else
        cone_ir = varargin{2};    % Ad SPECIFIED
    end
elseif (nargin == 5)
    cone_h = varargin{1};    % Ad SPECIFIED
    cone_ir = varargin{2};    % Ad SPECIFIED
    showplot = varargin{3};
end

% Generate extrusion data
h_out = min(cone_h,cone_or*tand(cone_th));
x_out = max(0,cone_or-h_out/tand(cone_th));
h_in = min(h_out,cone_ir*tand(cone_th));
x_in = max(0,cone_ir-h_in/tand(cone_th));

xy_data = [...
    cone_ir 0;
    cone_or 0;
    x_out h_out];

% Logic to avoid duplicate coordinates
if (x_out ~= 0)
    xy_data = [xy_data; x_in h_out];
end

if ((cone_ir ~= 0)  && (h_in ~= h_out))
    xy_data = [xy_data; x_in h_in];
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
    %    maxd = max(box_ox,box_oy);
    
    % Plot data
    patch(xy_data(:,1),xy_data(:,2),[1 1 1]*0.90,'EdgeColor','none');
    hold on
    plot(xy_data(:,1),xy_data(:,2),'-','Marker','o','MarkerSize',4,'LineWidth',2);
    
    % Mirror data to show revolution
    patch(-xy_data(:,1),xy_data(:,2),[1 1 1]*0.90,'EdgeColor','none');
    hold on
    plot(-xy_data(:,1),xy_data(:,2),'-','Marker','o','MarkerSize',4,'LineWidth',2);
    
    axis('equal');
    
    % Show parameters
    hold on
    
    plot([0 cone_or],[0 0],'r-d','MarkerFaceColor','r');
    text(0.25*cone_or,0.025*h_out,'{\color{red}cone\_or}','HorizontalAlignment','center');

    if(cone_ir>0)
    plot([0 -cone_ir],[0 0],'g-d','MarkerFaceColor','g');
    text(-0.25*cone_ir,0.025*h_out,'{\color{green}cone\_ir}','HorizontalAlignment','center');
    end
    
	plot([0 0],[0 cone_h],'b-d','MarkerFaceColor','b');
    text(0, 0.5*cone_h,'{\color{blue}cone\_h}');

    arc1_r = 0.6*cone_or;
    arc1 = [(0:1:cone_th)]'*pi/180;
    plot(-cos(arc1)*arc1_r+cone_or,sin(arc1)*arc1_r,'k-');
    plot(-cos(arc1(1))*arc1_r+cone_or,sin(arc1(1))*arc1_r,'kd','MarkerFaceColor','k');
    plot(-cos(arc1(end))*arc1_r+cone_or,sin(arc1(end))*arc1_r,'kd','MarkerFaceColor','k');
    text(-cos(cone_th/2*pi/180)*arc1_r*1.1+cone_or,sin(cone_th/2*pi/180)*arc1_r*1.1,'cone\_theta');


  	title('[xy\_data] = Extr\_Data\_Frustum\_Conical(cone\_or, cone\_th, <cone\_h, cone\_ir>);');
    hold off
    box on
    clear xy_data
end

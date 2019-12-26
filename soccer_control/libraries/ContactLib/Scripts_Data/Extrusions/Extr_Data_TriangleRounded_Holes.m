function [xy_data] = Extr_Data_TriangleRounded_Holes(L, H, xL, ri, ro, varargin)
%Extr_Data_TriangleRounded_Holes  Produce extrusion data for a triangle
%with rounded corners and a hole at each corner. with an arbitrary
%   [xy_data] = Extr_Data_TriangleRounded_Holes(L, W, r, h)
%   This function returns x-y data for the triangle.
%   You can specify:
%       Distance between lower holes     L
%       Distance from line connecting    H
%        lower holes to uppper hole     
%       Hole inner radius                ri
%       Triangle corner radius           ro
%       Top hole offset (0-1)*L          xL
%
%   To see a plot showing parameter values, enter the name
%   of the function with no arguments
%   >> Extr_Data_TriangleRounded_Holes
%
%   To see a plot created with your parameter values,
%   add 'plot' as the final argument
%   >> Extr_Data_TriangleRounded_Holes(6,4,0.33,1,2,'plot')
%
% Copyright 2014-2019 The MathWorks, Inc.

% Default data to show diagram
if (nargin == 0)
    L = 6;
    H = 4;
    ri = 1;
    ro = 1.5;
    xL = 0.33;
end

% Check if plot should be produced
if (isempty(varargin))
    showplot = 'n';
else
    showplot = char(varargin);
end

% Calculate lengths and angles
ang_R2H = atan2d(H,(xL*L));
ang_F2H = atan2d(H,((1-xL)*L));

xy_data1 = [L -ro];
xy_data2 = [cosd(270:-1:-90); sind(270:-1:-90)]'*ri+[L 0];
xy_data3 = [cosd(-90:1:90-ang_F2H); sind(-90:1:90-ang_F2H)]'*ro+[L 0];
xy_data4 = [cosd(90-ang_F2H); sind(90-ang_F2H)]'*ro+[xL*L H];
xy_data5 = [cosd(90-ang_F2H:-1:(90-ang_F2H-360)); sind(90-ang_F2H:-1:(90-ang_F2H-360))]'*ri+[xL*L H];
xy_data6 = [cosd(90-ang_F2H:1:(ang_R2H+90)); sind(90-ang_F2H:1:(ang_R2H+90))]'*ro+[xL*L H];
xy_data7 = [cosd(ang_R2H+90); sind(ang_R2H+90)]'*ro;
xy_data8 = [cosd((ang_R2H+90):-1:(ang_R2H-270)); sind((ang_R2H+90):-1:(ang_R2H-270))]'*ri;
xy_data9 = [cosd((ang_R2H-270):1:-90); sind((ang_R2H-270):1:-90)]'*ro;

xy_data = [xy_data1; xy_data2; xy_data3; xy_data4; xy_data5;xy_data6; xy_data7;xy_data8;xy_data9];

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
    
    % Show parameters
    plot([0 L],[0 0],'r-d','MarkerFaceColor','r');
    text(L/2,H/15,'{\color{red}L}');
    plot([1 1]*xL*L,[0 H],'b-d','MarkerFaceColor','b');
    text(xL*L*1.1,H/2,'{\color{blue}H}');
    plot([0 ri*sin(30*pi/180)],[0 ri*cos(30*pi/180)],'k-d','MarkerFaceColor','k');
    text(ri*sin(30*pi/180)*0.5,0.5*ri*cos(30*pi/180),'ri');
    plot([0 ro*cosd(-180)],[0 ro*sind(-180)],'g-d','MarkerFaceColor','g');
    text(ri*cosd(-180)*0.5,H/15,'{\color{green}ro}');
    plot([0 xL*L],[-1 -1]*ri/2,'m-d','MarkerFaceColor','m');
    text(xL*L/2,-ri/2-H/15,'{\color{magenta}xL*L}');
    
    title('[xy\_data] = Extr\_Data\_TriangleRounded\_Holes(L, H, ri, ro, xL);');
    hold off
    box on
    clear xy_data
end



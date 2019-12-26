function [xy_data] = Extr_Data_Cam_Roller_Curve(cam_xsec,roller_rad,inout,varargin)
%Extr_Data_Cam_Roller_Curve Produce curve data for a constraint that holds
%a circular roller to an arbitrary cam profile
%   [xy_data] = Extr_Data_Cam_Roller_Curve(cam_xsec,roller_rad,inout)
%   Produce curve data for a constraint that holds a circular roller to an
%   arbitrary cam profile
%
%   You can specify:
%       Outer profile of cam                   cam_xsec
%       Radius of roller                       roller_rad
%       Roller placement on profile            inout (Inside, Outside)
%
%   To see a plot showing parameter values, enter the name
%   of the function with no arguments
%   >> Extr_Data_Cam_Roller_Curve
%
%   To see a plot created with your parameter values,
%   add 'plot' as the final argument
%   >> Extr_Data_Cam_Roller_Curve([sind(1:4:360)' 1.5*cosd(1:4:360)'],0.2,'Outside','plot')

% Copyright 2017-2019 The MathWorks, Inc.

% Default data to show diagram
if (nargin == 0)
    roller_rad = 0.02;        % radius
    
    N = 100; % Number of sample points
    q = linspace(1/N,1,N)*2*pi;  % Angles cam points
    r = 4+sin(2*q)+0.3*cos(3*q); % Radius cam points
    c = cos(q);
    s = sin(q);
    cam_xsec = [r'.*c' r'.*s']/100;  % Cam profile
    
    inout = 'outside';
end

% Check if plot should be produced
if (isempty(varargin))
    showplot = 'n';
else
    showplot = varargin;
end

% Assumes counterclockwise sequence of points
if(strcmpi(inout,'inside'))
    normdir = -1;
else
    normdir = 1;
end

% Calculate path for roller constraint
% Calculate vector along each line segment
cam_segment_vec = [...
    circshift(cam_xsec(:,1),-1) - cam_xsec(:,1)  ...
    circshift(cam_xsec(:,2),-1) - cam_xsec(:,2)];

% Calculate length of each line segment
cam_segment_length = sqrt(...
    cam_segment_vec(:,1).^2 + cam_segment_vec(:,2).^2);

% Calculate unit vector along each line segment
cam_segment_uvec = cam_segment_vec./cam_segment_length;

% Determine vector normal to line segment
% Assumes counterclockwise sequence of points
cam_segment_norm_uvec = normdir*[cam_segment_uvec(:,2) -cam_segment_uvec(:,1)]; 

% Determine vector normal to curve at point
% by adding normal vectors of adjacent line segments
cam_point_norm_vec = cam_segment_norm_uvec+circshift(cam_segment_norm_uvec,-1);

% Obtain length of vector normal to curve at point
cam_point_norm_vec_length = sqrt(...
    cam_point_norm_vec(:,1).^2 + cam_point_norm_vec(:,2).^2);

% Obtain unit vector normal to curve at point
cam_point_norm_uvec = cam_point_norm_vec./cam_point_norm_vec_length;

% To each point on original curve, add a vector
% that is normal to curve at point 
% with length equal to radius of roller
xy_data = cam_xsec + circshift(cam_point_norm_uvec,1)*roller_rad;

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
    
    temp_colororder = get(gca,'defaultAxesColorOrder');
    
    % Plot extrusion data
    patch(cam_xsec(:,1),cam_xsec(:,2),[1 1 1]*0.90,'EdgeColor','none');
    hold on
    h2 = plot(xy_data(:,1),xy_data(:,2),'-','Marker','o','MarkerSize',4,'LineWidth',2);
    h3 = plot(cam_xsec(:,1),cam_xsec(:,2),'.','MarkerSize',6,'MarkerEdgeColor',[1 1 1]*0.70);
    h4 = plot(0,0,'+','MarkerEdgeColor','k');
    roller_patch_avec = linspace(1/50,2*pi,50)';
    roller_patch_data = ...
        [sin(roller_patch_avec) cos(roller_patch_avec)]*roller_rad + repmat(xy_data(1,:),length(roller_patch_avec),1);
    patch(roller_patch_data(:,1),roller_patch_data(:,2),temp_colororder(1,:),'EdgeColor','none','FaceAlpha',0.3);
    axis('equal');
    hold off
    box on
    
    % Show parameters
    hold on
    h5 = plot([cam_xsec(1,1) xy_data(1,1)],[cam_xsec(1,2) xy_data(1,2)],'k-d','MarkerFaceColor','k','MarkerSize',4);
    hold off

    title('[xy\_data] = Extr\_Data\_Cam\_Roller\_Curve(cam\_xsec,roller\_rad,inout);');
    legend([h2 h3],{'Roller Curve Points','Cam Profile Points'},'Location','Best')

    clear xy_data
end

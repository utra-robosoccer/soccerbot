function Extr_Data_Custom(xy_data)
%Extr_Data_Custom 	Plot custom extrusion data.
%   This function plots x-y data for a custom extrusion.

% Copyright 2012-2019 The MathWorks, Inc.

% Plot diagram to show parameters and extrusion

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
title(['Extr\_Data\_Custom(xy\_data);']);

axis('equal');

hold off
box on
clear xy_data

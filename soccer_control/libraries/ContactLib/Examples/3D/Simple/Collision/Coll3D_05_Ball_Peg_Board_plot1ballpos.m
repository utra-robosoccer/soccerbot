% Code to plot simulation results from Coll3D_05_Ball_Peg_Board
%
% Copyright 2015-2019 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(h1_Coll3D_05_Ball_Peg_Board)
catch
    h1_Coll3D_05_Ball_Peg_Board=figure('Name','Coll3D_05_Ball_Peg_Board');
end

% Generate simulation results if they don't exist
if(~exist('simlog_Coll3D_05_Ball_Peg_Board','var'))
    sim('Coll3D_05_Ball_Peg_Board')
end

% Get simulation results
temp_b1px = simlog_Coll3D_05_Ball_Peg_Board.Bushing_Joint.Px.p.series;
temp_b1py = simlog_Coll3D_05_Ball_Peg_Board.Bushing_Joint.Py.p.series;

temp_xcir = sin(0:0.2:2*pi)*peg.rad;
temp_ycir = cos(0:0.2:2*pi)*peg.rad;

% Plot board
patch([-1 1 1 -1]*platform.width/2,[-1 -1 1 1]*platform.length/2,[1 1 1]*0.8);
hold on

% Plot ball path
plot(temp_b1py.values,-temp_b1px.values,'r','LineWidth',1);

% Plot peg board
temp_pegxy = [0 -1;0 1; -1 0;1 0;-3 2;-1 2;1 2;3 2]*peg_spacing;
for i=1:size(temp_pegxy,1)
    patch(temp_xcir+temp_pegxy(i,1)*ones(size(temp_xcir)),...
          -(temp_ycir+temp_pegxy(i,2)*ones(size(temp_xcir))),...
          [0.0 0.5 0.7]);
end
temp_arcvector = [pi/2:0.2:3*pi/2];
temp_arcctr = [0 -2;-2 -2;2 -2]*peg_spacing;
for i=1:size(temp_arcctr,1)
plot(sin(temp_arcvector)*peg_spacing+temp_arcctr(i,1)*ones(size(temp_arcvector)),...
    cos(temp_arcvector)*peg_spacing+temp_arcctr(i,2)*ones(size(temp_arcvector)),...
    'Color',[0.0 0.5 0.7],'LineWidth',3);
end
hold off

box on
axis([[-1 1]*platform.width/2 [-1 1]*platform.length/2]*1.1);
axis equal
title('Ball Path on Peg Board');

% Remove temporary variables
clear temp_b1px temp_b1py temp_xcir temp_pegxy temp_ycir temp_arcvector temp_arcctr


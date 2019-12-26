%% Circle to Circle Contact Force (2D)
% 
% This block implements a 2D contact force between two circles. It acts to
% repel frames CirB and CirF.
% 
% This is part of the <matlab:web('Contact_Forces_Demo_Script.html'); Simscape Multibody Contact Forces Library>
%
% Copyright 2014-2019 The MathWorks, Inc.



%% 
% 
% <<Circle_to_Circle_Help_IMAGE.png>>

%% 
%
% Frames connected to ports: 
% 
% # Located at center of circle 
% # Circle is in XY plane of frame 
% # Active range: counter-clockwise from -x axis of CirB
% 
% Output signal is a bus with intermediate calculations and total force.
% 

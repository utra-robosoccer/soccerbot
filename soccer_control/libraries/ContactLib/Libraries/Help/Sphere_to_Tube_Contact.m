%% Sphere to Tube Contact Force (3D)
% 
% This block implements a contact force between a sphere and a tube. The
% force is active outside and inside the tube.
% 
% This is part of the <matlab:web('Contact_Forces_Demo_Script.html'); Simscape Multibody Contact Forces Library>
%
% Copyright 2014-2019 The MathWorks, Inc.



%% 
% 
% <<Sphere_to_Tube_Help_IMAGE.png>>

%% 
%
% Frame connected to TubB port:
%
% # Located at midpoint of the tube axis
% # Z-axis is aligned with the tube axis
% # Active range: counter-clockwise from -x axis of TubB
% 
% Frame connected to SphF port:
%
% # Located at center of sphere
% # Orientation does not matter
% 
% Output signal is a bus with intermediate calculations and total force.
% 
% 

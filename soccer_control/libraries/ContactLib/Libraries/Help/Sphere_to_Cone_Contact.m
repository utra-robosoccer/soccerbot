%% Sphere to Cone Contact Force (3D)
% 
% This block implements a contact force between a sphere and a cone. The
% force is active outside and inside the cone.
%
% This is part of the <matlab:web('Contact_Forces_Demo_Script.html'); Simscape Multibody Contact Forces Library>
%
% Copyright 2014-2017 The MathWorks, Inc.



%% 
% 
% <<Sphere_to_Cone_Help_IMAGE.png>>

%% 
%
% 
% Frame connected to ConB port:
% 
% # Located at widest end of the cone
% # Z-axis is aligned with the cone axis
% # Active range: counter-clockwise from -x axis of ConB
% 
% Frame connected to SphF port:
% 
% # Located at center of sphere
% # Orientation does not matter
% 
% Output signal is a bus with intermediate calculations and total force.
% 
% 

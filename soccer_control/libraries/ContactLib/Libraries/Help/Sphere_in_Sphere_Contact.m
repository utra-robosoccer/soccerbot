%% Sphere in Sphere Contact Force (3D)
% 
% This block implements a contact force between two spheres.  The force
% acts to keep distance from frame SphB to SphF less than (inner radius
% SphB - radius SphF).
%
% This is part of the <matlab:web('Contact_Forces_Demo_Script.html'); Simscape Multibody Contact Forces Library>
%
% Copyright 2014-2017 The MathWorks, Inc.



%% 
% 
% <<Sphere_in_Sphere_Help_IMAGE.png>>

%% 
%
% Frames connected to ports:
%
% # Located at center of sphere
% # Orientation does not matter
% 
% Output signal is a bus with intermediate calculations and total force.
% 
% 

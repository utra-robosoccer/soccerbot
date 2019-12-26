%% Sphere to Plane Contact Force (3D)
% 
% This block implements a contact force between a sphere and a plane.  The
% force is active above and below the plane.
% 
% This is part of the <matlab:web('Contact_Forces_Demo_Script.html'); Simscape Multibody Contact Forces Library>
%
% Copyright 2014-2019 The MathWorks, Inc.



%% 
% 
% <<Sphere_to_Plane_Help_IMAGE.png>>

%% 
%
% Frame connected to PlaB port:
%
% # Located at midpoint of plane (x, y, and z).
% # Z-axis is normal to the surfaces where force is active. 
% 
% Frame connected to the SphF port:
% 
% # Located at center of sphere.
% # Orientation does not matter.
% 
% Output signal is a bus with intermediate calculations and total force.
% 
% 

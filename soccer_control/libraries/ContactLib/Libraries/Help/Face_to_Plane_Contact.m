%% Face to Plane Contact Force (3D)
% 
% This subsystem implements a contact force between a plane and a square
% face. The plane is assumed significantly larger than the face such that
% the edges will never intersect.  The force is active above and below the
% plane and face.
%
% This is part of the <matlab:web('Contact_Forces_Demo_Script.html'); Simscape Multibody Contact Forces Library>
%
% Copyright 2014-2017 The MathWorks, Inc.



%% 
% 
% <<Face_to_Plane_Help_IMAGE.png>>

%% 
%
% 
% Frame connected to PlaB port:
%
% # Located at midpoint of plane (x, y, and z).
% # Z-axis is normal to the surfaces where force is active. 
% 
% Frame connected to the FacF port:
%
% # Located at midpoint of box (x, y, and z).
% # Z-axis is normal to the surfaces where force is active. 
% 
% Output signal is a bus with intermediate calculations and total force.
% 
% 

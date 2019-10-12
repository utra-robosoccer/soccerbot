%% Circle to Finite Line Contact Force (2D)
% 
% This block implements a 2D contact force between a circle and a finite
% line.  The force is active above and below the line.
% 
% This is part of the <matlab:web('Contact_Forces_Demo_Script.html'); Simscape Multibody Contact Forces Library>
%
% Copyright 2014-2017 The MathWorks, Inc.



%% 
% 
% <<Circle_to_Finite_Line_Help_IMAGE.png>>

%% 
%
% Frame connected to LinB port: 
% 
% # Located at midpoint of line. 
% # X-axis is normal to the finite line. 
% # Y-axis is along the finite line.
% 
% Frame connected to the CirF port: 
%
% # Z-axis aligned with frame attached to LinB port.
% 
% Output signal is a bus with intermediate calculations and total force.
% 

%% Circle to Ring Contact Force (2D)
% 
% This block implements a 2D contact force between a circle and a ring.
% The force is active on the outside and the inside of the ring.
% 
% This is part of the <matlab:web('Contact_Forces_Demo_Script.html'); Simscape Multibody Contact Forces Library>
%
% Copyright 2014-2019 The MathWorks, Inc.



%% 
% 
% <<Circle_to_Ring_Help_IMAGE.png>>

%% 
%
% Frames connected to ports:
% 
% # Located at center of circle/ring
% # Circle/Ring is in XY plane of frame
% # Active range: counter-clockwise from -x axis of RinB
% 
% Output signal is a bus with intermediate calculations and total force.
% 
% 

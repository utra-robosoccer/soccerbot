% CAM PARAMETERS
% Copyright 2014-2019 The MathWorks, Inc.

Cam_PARAM.Cam.Radius_rev = 2; % cm
Cam_PARAM.Cam.Radius_tip = 1; % cm
Cam_PARAM.Cam.Dist_ctrs = 2;  % cm
Cam_PARAM.Cam.Thickness = 1;  % cm
Cam_PARAM.Cam.Theta1 = acos((Cam_PARAM.Cam.Radius_rev-Cam_PARAM.Cam.Radius_tip)/Cam_PARAM.Cam.Dist_ctrs)*180/pi;

Cam_PARAM.Roller.Radius = 0.75;  % cm
Cam_PARAM.Roller.Thickness = 2;  % cm

%Extr_Data_Ring(1.5,0,0,90,'plot')

Cam_PARAM.Follower.Spring_k = 10;  % N/m
Cam_PARAM.Follower.Damper_b = 1;  % N/(m/s)

Cam_PARAM.Follower.Length = 7.25;  % cm
Cam_PARAM.Follower.Shaft_Rad = 0.3;  % cm

Cam_PARAM.Follower.Housing.Length = 2;  % cm
Cam_PARAM.Follower.Housing.Rad = 0.5;  % cm

Cam_PARAM.ContactCF.Stiffness = 1e3;
Cam_PARAM.ContactCF.Damping = 10;
Cam_PARAM.ContactCF.Filter = 1e-4;

Cam_PARAM.Rocker.Pushrod.Offset= 3;
Cam_PARAM.Rocker.Pushrod.End_rad = 1;
Cam_PARAM.Rocker.Pushrod.Thickness = 1;
Cam_PARAM.Rocker.Center.Rad = 1.25;
Cam_PARAM.Rocker.Valve.Offset = 6;
Cam_PARAM.Rocker.Valve.End_rad = 0.5;
Cam_PARAM.Rocker.Pivot_rad = 0.5;
Cam_PARAM.Rocker.Pivot_len = 2;
Cam_PARAM.Rocker.Pivot_camsh_offset = 10;


Cam_PARAM.Valve.Stem.Radius = 0.15;
Cam_PARAM.Valve.Stem.Length = 10;
Cam_PARAM.Valve.Head.Radius = 1;
Cam_PARAM.Valve.Head.Height = 0.5;
Cam_PARAM.Valve.Head.Thickness = 0.1;

extr1 = Extr_Data_Ellipse(Cam_PARAM.Valve.Head.Radius,Cam_PARAM.Valve.Head.Height,180,270,1);
extr2 = extr1+repmat([Cam_PARAM.Valve.Head.Radius -Cam_PARAM.Valve.Stem.Length],size(extr1,1),1);
extr3 = [0 0;extr2;extr2(end,:)+[0 -Cam_PARAM.Valve.Head.Thickness]];
Cam_PARAM.Valve.Extr_Data = flipud([0 0;extr3+repmat([Cam_PARAM.Valve.Stem.Radius 0],size(extr3,1),1);[0 extr3(end,2)]]);
clear extr1 extr2 extr3

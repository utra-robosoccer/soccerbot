% Copyright 2017-2019 The MathWorks, Inc.

% Bearing geometry
bearing.D           = 0.1;      % Outer diameter (m)
bearing.d           = 0.04;     % Inner diameter (m)
bearing.b           = 0.02;     % Width (m)
bearing.rsh         = 0.002;    % Chamfer radius (m)
bearing.nth         = 35;       % Half of sweep angle of race channel (deg)
bearing.ball.rad    = 0.0075;   % Radius of ball (m)

% Create the cross section profile for the races in the bearing.
% These calculations assume that the ball is exactly between the outer
% radius and the inner radius of the bearing.

temp_outer_race_profile = Extr_Data_Block_NotchCircle(...
    bearing.b,...
    (bearing.D/4-bearing.d/4-bearing.ball.rad*cosd(bearing.nth)),...
    bearing.ball.rad,...
    bearing.nth,...
    bearing.rsh);
bearing.outer_race.xc = rot90(temp_outer_race_profile + ...
    [0 (bearing.D+bearing.d)/4+bearing.ball.rad*cosd(bearing.nth)],2);

temp_inner_race_profile = flipud(temp_outer_race_profile).*[1 -1];

bearing.inner_race.xc = rot90(temp_inner_race_profile + ...
    [0 (bearing.D+bearing.d)/4-bearing.ball.rad*cosd(bearing.nth)],2);
%{
Extr_Data_Custom(...
    [bearing.outer_race.xc;
     bearing.inner_race.xc;
     bearing.inner_race.xc(1,:);
     bearing.outer_race.xc(end,:)]);
%}
    
% Inner and outer race parameters
bearing.outer_race.rho = 7000;
bearing.outer_race.clr = [0.8 0.8 0.5];
bearing.outer_race.opc = 1;

bearing.outer_race.mark_clr = [0.6 0.6 0.4];
bearing.outer_race.mark_opc = 1;
bearing.outer_race.mark_xc = ...
    [1.08 1; 1.08 -1; 1.12 -1; 1.12 1].* ...
    [(bearing.D+bearing.d)/4+bearing.ball.rad bearing.b*0.51];

bearing.inner_race.rho = 7000;
bearing.inner_race.clr = [0.8 0.8 0.5];
bearing.inner_race.opc = 1;

bearing.inner_race.mark_clr = [0.6 0.6 0.4];
bearing.inner_race.mark_opc = 1;
bearing.inner_race.mark_xc = ...
    [0.90 1; 0.90 -1; 0.97 -1; 0.97 1].* ...
    [(bearing.D+bearing.d)/4-bearing.ball.rad bearing.b*0.51];

% Ball parameters
bearing.ball.rho        = 7000;
bearing.ball.cage_b     = 1e-8;
bearing.ball.clr        = [0.6 0.6 0.6];
bearing.ball.opc        = 1;
bearing.ball.mark.clr   = [0.9 0.9 0.9];
bearing.ball.mark.opc   = 1;

% Cage parameters
bearing.cage.height = bearing.ball.rad*(cosd(bearing.nth))*2*0.5;
bearing.cage.rad_i  = bearing.ball.rad*1.001;
bearing.cage.rad_o  = bearing.cage.rad_i*1.1;
bearing.cage.thk  = (bearing.cage.rad_o-bearing.cage.rad_i)*2;
bearing.cage.loop_xc = [...
    bearing.cage.rad_o -bearing.cage.height/2;
    bearing.cage.rad_o  bearing.cage.height/2;
    bearing.cage.rad_i  bearing.cage.height/2;
    bearing.cage.rad_i -bearing.cage.height/2];

bearing.cage.web.outerang = ...
    360/8/2-atan2d(bearing.cage.rad_o,(bearing.D+bearing.d)/4+bearing.cage.height/2);
bearing.cage.web.innerang = ...
    360/8/2-atan2d(bearing.cage.rad_o,(bearing.D+bearing.d)/4-bearing.cage.height/2);
bearing.cage.web.centerang = ...
    360/8/2-atan2d(bearing.cage.rad_o,(bearing.D+bearing.d)/4);

bearing.cage.web.outerrad = sqrt( ...
((bearing.D+bearing.d)/4+bearing.cage.height/2)^2 + bearing.cage.rad_o^2);
bearing.cage.web.innerrad = sqrt( ...
((bearing.D+bearing.d)/4-bearing.cage.height/2)^2 + bearing.cage.rad_o^2);
bearing.cage.web.centerrad = (bearing.cage.web.outerrad+bearing.cage.web.innerrad)/2;

bearing.cage.web_xc = ...
    [[cosd(linspace(-bearing.cage.web.outerang,bearing.cage.web.outerang,25))' ...
      sind(linspace(-bearing.cage.web.outerang,bearing.cage.web.outerang,25))']* ...
       bearing.cage.web.outerrad;
     [cosd(linspace(bearing.cage.web.innerang,-bearing.cage.web.innerang,25))' ...
      sind(linspace(bearing.cage.web.innerang,-bearing.cage.web.innerang,25))']* ...
      bearing.cage.web.innerrad] ...
       -[bearing.cage.web.centerrad 0];
  
bearing.cage.rho = 7000;
bearing.cage.clr = [1 1 1]*0.9;
bearing.cage.opc = 1;

    
% Contact force parameters
bearing.ball.k = 1e5;
bearing.ball.b = 1e3;
bearing.ball.muk = 0.7;
bearing.ball.mus = 0.9;
bearing.ball.vth = 1e-2;
bearing.ball.preload = 5;
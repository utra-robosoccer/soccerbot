function [dist_to_floor,isterminal,direction] = theta_impact_event(t,x,theta_final)
%For both swing and stance rocker feet
theta = x(1);

%Add tolerance
dist_to_floor = theta-theta_final;
isterminal = 1;  % Halt integration
direction = 0;   % The zero can be approached from either direction

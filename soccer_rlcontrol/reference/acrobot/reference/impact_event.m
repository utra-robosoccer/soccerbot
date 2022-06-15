function [dist_to_floor,isterminal,direction] = impact_event(t,x,l)
%For both swing and stance rocker feet
q1 = x(1);
q2 = x(2);
l1 = l(2);
l2 = l(4);


rH = l1*[cos(q1);sin(q1)];
P_heel2 = rH + l2*[cos(q1+q2); sin(q1+q2)];

swing_foot_ahead = P_heel2(1)>0;

dist_to_floor = P_heel2(2); %Value we want to be 0

%Add tolerance
dist_to_floor = dist_to_floor;
isterminal = 1;  % Halt integration
direction = 0;   % The zero can be approached from either direction

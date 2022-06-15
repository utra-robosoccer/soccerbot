function [dist,isterminal,direction] = q_b_intersect_event(theta,q,q_b)
%Intersect with S_minus

%Note: need sign to trigger crossing event; since intersetions with
%horizontal move left to right with increase in u, this distance changes
%monotonically with u

% euclid_dist = norm(q-q_b);
% sign_d = sign(q(1)-q_b(1));%consider left of q_b as positivet
dist = q(2)-q_b(2) ;

isterminal = 1;  % Halt integration
direction = 0;   % The zero can be approached from either direction

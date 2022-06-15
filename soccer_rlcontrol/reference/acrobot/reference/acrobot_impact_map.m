function [ x0_new ] = acrobot_impact_map(x,lz,mz,calc_De,calc_E2,calc_dUpsilon_e_dqs )
%Calculates the impact map for rocker foot model
N=2;
q1 = x(1); %pre-impact
q2 = x(2); %pre-impact

qs = [q1; q2];
q1_dot_m = x(3);
q2_dot_m = x(4);
qs_dot= [q1_dot_m; q2_dot_m];

lc1 = lz(1);
l1 = lz(2);
lc2 = lz(3);
l2 = lz(4);


m1 = mz(1);
mH = mz(2);
m2 = mz(3);


De = calc_De(l1,lc1,lc2,m1,m2,q1,q2);
E2 = calc_E2(l1,l2,q1,q2);
dUpsilon_e_dqs = calc_dUpsilon_e_dqs();

%qs_p = qs_m; delta_qedot gives qs_dot_p (all before relabelling)
last_term = [eye(N); dUpsilon_e_dqs];

delta_F = -((E2/De)*transpose(E2))\E2*last_term;
delta_qedot = (De\transpose(E2))*delta_F + last_term;

%Relabelling
T = [1 1; 0 -1];
delta_qsdot = [T zeros(N,2)]*delta_qedot;


%Relabelled
q_p_r = T*qs + [-pi; 0];
qdot_p_r = delta_qsdot*qs_dot;


x0_new = [q_p_r ; qdot_p_r];
end

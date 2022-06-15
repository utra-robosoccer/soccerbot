function dxdt = acrobot(t,x,l,m,B,calc_D,calc_b1, beta, Kp,Kd,g_func)

%%Using solve_EL_v2
dxdt = zeros(6,1);

q1 = x(1);
q2 = x(2);
q1dot = x(3);
q2dot = x(4);

%Physical parameters
g = 9.81;
m1 = m(1);
mH = m(2);
m2 = m(3);

lc1 = l(1);
l1 = l(2);
lc2 = l(3);
l2 = l(4);


%% Model

qdot = [q1dot; q2dot];
D = calc_D(l1,lc1,lc2,m1,m2,q2);
% b1 = -C*qdot - G
b1 = calc_b1(g,l1,lc1,lc2,m1,m2,q1,q2,q1dot,q2dot);

g_prime = fnder(g_func,1);
g_d_prime = fnder(g_func,2);

g_val = ppval(g_func,q1);
g_prime_val = ppval(g_prime,q1);
g_d_prime_val = ppval(g_d_prime,q1);

e = q2 - g_val;
e_dot = q2dot - g_prime_val*q1dot;

term1 = inv([-g_prime_val 1]*inv(D)*B);
term2 = (-Kp*e - Kd*e_dot + g_d_prime_val*q1dot^2 + [-g_prime_val 1]*inv(D)*(-b1));

Tau_star = term1*term2;

Tau = Tau_star
% Tau = 0; %no controller

q_ddot = D\(B*Tau) + D\b1;


dxdt = [qdot; q_ddot];


end

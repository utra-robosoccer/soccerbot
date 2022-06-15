%% Symbolic computations for rocker_ankle_main
clc; clear;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                         PART 1: SYMBOLIC EOMS                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%Setup
syms q1 q2 q1dot q2dot q1d_dot q2d_dot
syms l1 lc1 l2 lc2
syms m1 m2 g

q = [q1; q2];
qdot = [q1dot; q2dot];
qd_dot = [q1d_dot; q2d_dot];


%% Relevant points

%Centers of mass
rc1 = lc1*[cos(q1); sin(q1)];
rH = l1*[cos(q1);sin(q1)];
rc2 = rH + lc2*[cos(q1+q2); sin(q1+q2)];

%Angular velocities
w_1_0 = [0;0;1]*(q1dot);
w_2_0 = [0;0;1]*(q1dot+q2dot);


rc1_dot = simplify(jacobian(rc1,q)*qdot);
rc2_dot = simplify(jacobian(rc2,q)*qdot);


%% Kinetic energy
Ta = 0.5*m1*(transpose(rc1_dot)*rc1_dot);
Tb = 0.5*m2*(transpose(rc2_dot)*rc2_dot);

T = Ta + Tb;
%% Potential energy
Ua = m1*g*rc1(2);
Ub = m2*g*rc2(2);

U = Ua + Ub;


%% Lagrangian
L = T - U;

dLdq = transpose(jacobian(L,q));
dLdqdot = transpose(jacobian(L,qdot));

ddt_dLdqdot = jacobian(dLdqdot,q)*qdot + jacobian(dLdqdot,qdot)*qd_dot;
%% Find EOM and simplify expressions

%Below, EOM = B*Tau
EOM = ddt_dLdqdot - dLdq; %B*Tau = D*qd_dot + C*qdot + G

EOM_simp = simplify(EOM);
EOM1 = EOM_simp(1);
EOM2 = EOM_simp(2);

%%
%Solve for D matrix;
%Gives D*qddot = b1, b1 = -C*qdot - G
[D, b1] = equationsToMatrix(EOM_simp, [q1d_dot; q2d_dot]);
%%
%Solve for D matrix
calc_D = matlabFunction(D);
calc_b1 = matlabFunction(b1);

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          PART 2: CONTROLLER                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MOVED TO SEPARATE FILE

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          PART 3: IMPACT MAP                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%Solving for EOM and impact map following
% "Feedback Control of Dynamic Bipedal Robot Locomotion" by Grizzle, p. 55
% Based on the 3 link model on p. 67 and the paper "Asymptotically Stable
% Walking for Biped Robots: Analysis via Systems with Impulse Effects"

syms q3 q4 %q3, q4 are coordinates of reference frame of stance leg
syms q3dot q4dot
syms q3d_dot q4d_dot


%extended vector (detached foot)
qe = [q1; q2; q3; q4];
qedot = [q1dot; q2dot; q3dot; q4dot];
qed_dot = [q1d_dot; q2d_dot; q3d_dot; q4d_dot];

qs = q;
qs_dot = qdot;
N = size(qs,1);

%% Values with detached coordinate system
%Centers of mass
rc1_d = lc1*[cos(q1); sin(q1)]+[q3;q4];
rH_d = l1*[cos(q1);sin(q1)]+[q3;q4];
rc2_d = rH + lc2*[cos(q1+q2); sin(q1+q2)]+[q3;q4];

rc1_dot_d = simplify(jacobian(rc1_d,qe)*qedot);
rc2_dot_d = simplify(jacobian(rc2_d,qe)*qedot);



%% Kinetic energy
Ta = 0.5*m1*(transpose(rc1_dot_d)*rc1_dot_d);
Tb = 0.5*m2*(transpose(rc2_dot_d)*rc2_dot_d);

T = Ta + Tb ;

%% Potential energy

Ua = m1*g*rc1_d(2);
Ub = m2*g*rc2_d(2);

U = Ua + Ub;

%% Lagrangian
Le = simplify(T - U);

dLe_dq = transpose(jacobian(Le,qe));
dLe_dqdot = transpose(jacobian(Le,qedot));

%%
ddt_dDLedqdot = jacobian(dLe_dqdot,qe)*qedot + jacobian(dLe_dqdot,qedot)*qed_dot;
ddt_dDLedqdot = simplify(ddt_dDLedqdot);

%% Find EOM and simplify expressions
Tau_e = ddt_dDLedqdot - dLe_dq;
Tau_e_simp = simplify(Tau_e);

%% Solve for D matrix
[De, be1] = equationsToMatrix(Tau_e_simp, [q1d_dot; q2d_dot; q3d_dot; q4d_dot]);
%% Impact map
%Position of impacting foot

P_heel2 = rH + l2*[cos(q1+q2); sin(q1+q2)] + [q3;q4];

%Upsilons
Upsilon_e = [q3;q4];
Upsilon2 = simplify(P_heel2);

E2 = jacobian(Upsilon2,qe);
dUpsilon2_dqs = jacobian(Upsilon2,qs);
dUpsilon_e_dqs = jacobian(Upsilon_e,qs);

E2 = simplify(E2);
dUpsilon_e_dqs = simplify(dUpsilon_e_dqs);

calc_De = matlabFunction(De);
calc_E2 = matlabFunction(E2);
calc_dUpsilon_e_dqs = matlabFunction(dUpsilon_e_dqs);

%Note: impact map computations are done online since delta_F is too slow to
%compute symbolically
%%
% sys = matlabFunction([qdot; qd_dot_expr], 'vars', {'t', [q; qdot]});
save('acrobot_syms.mat');
compute_h_line %Find the spline for the VHC curve

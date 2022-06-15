 %Test angles

clc; clear;
close all;
load('acrobot_syms.mat');
clearvars -except calc_De calc_E2 calc_dUpsilon_e_dqs calc_D

%CHECK: q3, q4 not needed??

%setup parameters
phi = 0;
N=2;
m = [3.2;20;3.2]; %m1 mH m2
l = [0.2;0.4;0.2;0.4]; %lc1 ll lc2 l2
B = [0;1];

lc1 = l(1);
l1 = l(2);
lc2 = l(3);
l2 = l(4);


m1 = m(1);
mH = m(2);
m2 = m(3);

%% VHC parameters
beta = 9*pi/10; %standing aperture
beta = pi/4;
%setup parameters

q_p = [(pi+beta)/2; pi-beta]; %post impact
q_m = [(pi-beta)/2; beta-pi]; %pre impact

D_p = calc_D(l1,lc1,lc2,m1,m2,q_p(2)); %D matrix immediately post impact
D_m = calc_D(l1,lc1,lc2,m1,m2,q_m(2)); %D matrix immediately pre impact
test_vec_m = inv(D_m)*B; %vector we want v to be transversal to
test_vec_p = inv(D_p)*B; %vector we want w to be transversal to
%%
delta_z = 1;

kappa_range = linspace(-5,pi,100);

angles = zeros(size(kappa_range));

for i =1:size(kappa_range,2)
    kappa = kappa_range(i);
    w = [cos(kappa);sin(kappa)]; %w matrix; dsigma/dtheta pre impact

    %x_m (pre-impact)
    q1 = q_m(1);
    q2 = q_m(2);
    q1_dot_m = w(1);
    q2_dot_m = w(2);

    qs_dot= [q1_dot_m; q2_dot_m];



    De = calc_De(l1,lc1,lc2,m1,m2,phi,q1,q2); %no q3, q4 dependence?
    E2 = calc_E2(l1,l2,phi,q1,q2);
    dUpsilon_e_dqs = calc_dUpsilon_e_dqs();

    %qs_p = qs_m; delta_qedot gives qs_dot_p (all before relabelling)
    last_term = [eye(N); dUpsilon_e_dqs];

    delta_F = -((E2/De)*transpose(E2))\E2*last_term;
    delta_qedot = (De\transpose(E2))*delta_F + last_term;

    %Relabelling
    T = [1 1; 0 -1];
    delta_qsdot = [T zeros(N,2)]*delta_qedot;


    %Relabelled
    q_p_r = T*q_m + [-pi; 0];
    v = (delta_qsdot)*w/delta_z;

    v_angle =atan2(v(2),v(1));
    angle = v_angle;
%     angle = kappa - w_angle;
    angles(i) = angle;

end
% figure
plot(kappa_range, angles);
xlabel('w angle (pre-impact)')
ylabel('v angle (post-impact)')

%%
hold on
eta = atan2(2,-1);
vertline = ones(100)*(eta-2*pi);
vertlinevals = linspace(-3,4,100);
vertline2 = vertline + pi;
vertline3 = vertline+2*pi;

plot(vertline,vertlinevals);
plot(vertline2, vertlinevals);
plot(vertline3, vertlinevals);

horline = linspace(-5,4,100);
horlinevals = ones(100)*(eta);
horlinevals2 = horlinevals-pi;


plot(horline,horlinevals);
plot(horline,horlinevals2);

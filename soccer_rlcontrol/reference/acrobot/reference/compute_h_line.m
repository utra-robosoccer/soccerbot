%% Script to find curve h using VCG
%Guarantees hybrid invariance and regularity (no guarantee of
%existence/uniqueness)

clc; clear;
close all;
load('acrobot_syms.mat');
clearvars -except calc_De calc_E2 calc_dUpsilon_e_dqs calc_D D

%% setup parameters
load_acrobot_params

calc_D_numeric = matlabFunction(subs(D));

%% VHC parameters

beta = pi/4;

%Hybrid invariance requirements
v_w_scale = 1; %Scaling between w and v
kappa = 2.1; %Angle of w, pre-impact
w = 1*[cos(kappa);sin(kappa)];


q_p = [(pi+beta)/2; pi-beta]; %post impact
q_m = [(pi-beta)/2; beta-pi]; %pre impact

D_p = calc_D(l1,lc1,lc2,m1,m2,q_p(2)); %D matrix immediately post impact
D_m = calc_D(l1,lc1,lc2,m1,m2,q_m(2)); %D matrix immediately pre impact
test_vec_m = inv(D_m)*B; %vector we want v to be transversal to
test_vec_p = inv(D_p)*B; %vector we want w to be transversal to

%% Impact map to find w

%x_m (pre-impact)
q1_m = q_m(1);
q2_m = q_m(2);


De = calc_De(l1,lc1,lc2,m1,m2,q1_m,q2_m);
E2 = calc_E2(l1,l2,q1_m,q2_m);
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
v = (delta_qsdot)*w/v_w_scale; %post-impact
v = 1*(v/norm(v)); %make v a unit vector

%Check that v = dsigma/dtheta post impact is transversal to inv(D)*B
v_check = det([v test_vec_p]);


%% DESIGN THE CURVE HERE!!!

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                    DESIGNING THE CURVE!!!  >:(                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Line parameters
r0 = 0.2; %start to q_p
r1 = 0.55;%q_p to qa
t0 = 0.05; %qb to q_m
t1 = 0.2; %q_m to end

%Number of samples
r0_samples = 2; %start to q_p
r1_samples = 3; %q_p to qa
s_samples = 10; %qa to qb
t0_samples = 6; %qb to q_m
t1_samples = 2; %q_m to end

%Truncating indices
%Note: without truncation there is overlap at qa and qb
r1_truncate = 1; %Truncate before qa
s_start_truncate = 4; %Truncate after qa
s_end_truncate = 5; %Truncate before qb
t0_truncate = 3;  %Truncate after qb


%% Creating the curve

%Creating all sample points
r0_vec = linspace(-r0,0,r0_samples);
r1_vec = linspace(0,r1,r1_samples);
r = [r0_vec(1:end-1), r1_vec]; %start to qa

t0_vec = linspace(-t0,0,t0_samples);
t1_vec = linspace(0,t1,t1_samples);
t = [t0_vec(1:end-1), t1_vec]; %qb to end

%Corner points
qa = q_p + v*r(end);
qb = q_m + w*t(1);

%Middle section
s = linspace(r1_vec(end),1-abs(t0_vec(1)),s_samples); %qa to qb
s_shifted = s-s(1); %Shifting to start at 0

mid_vec = (qb-qa)/(s(end)-s(1));

%Sampled theta, q to be truncated
theta_rough = [r, s, t+1];
q_start = q_p + v*r;
q_mid = qa + mid_vec*s_shifted;
q_end = q_m + w*t;
q_rough = [q_start, q_mid, q_end];


%Plotting
test_plot = figure;
hold on
plot(q_p(1), q_p(2),'.', 'MarkerSize',13,'color','k');
plot(q_m(1), q_m(2),'.', 'MarkerSize',13,'color','g');
plot(q_rough(1,:), q_rough(2,:), 'o', 'color', 'b')


%Find theta values for start and end of curve (intersecting S+, S-)
theta_start_index = find(r==0);
theta_start =theta_rough(theta_start_index);
theta_end_index = find(t==0)+size(r,2)+size(s,2);
theta_end = theta_rough(theta_end_index);



%Create breakpoints for spline
r_bar = r(1:end - r1_truncate);
t_bar = t(t0_truncate+1:end);
s_bar = s(s_start_truncate+1:end-s_end_truncate);


theta_truncated = [r_bar, s_bar, 1+t_bar];
q_truncated= [q_start(:,1:end-r1_truncate), q_mid(:,s_start_truncate+1:end-s_end_truncate), q_end(:,t0_truncate+1:end)];

figure(test_plot);
plot(q_truncated(1,:), q_truncated(2,:), '*');

%The curve
sigma = spline(theta_truncated, [v q_truncated w]);

test = linspace(theta_start,theta_end,800);
test2 = ppval(sigma,test);
figure(test_plot)
plot(test2(1,:), test2(2,:));


%% Plot the Rough solution curve

%Plot vector field inv(D)*B
q1_range = 0/16:.05:pi;
q2_range =-2*pi:0.05:2*pi;


[X1,X2] = meshgrid(q1_range,q2_range);


R = zeros(size(X1));
Z = zeros(size(X2));

%Dinv*B
for i=1:size(X2,1)
    for j=1:size(X2,2)
        temp = inv(calc_D(l1,lc1,lc2,m1,m2,X2(i,j)))*B;
        R(i,j) = temp(1);
        Z(i,j) = temp(2);
    end
end

VHC_plot = figure('Name','VHC_plot');
hold on
axis square
title('Plot of sigma(theta)')
xlabel('q1')
ylabel('q2')
% quiver(X1,X2,r,z) %vector field
streamslice(X1,X2,R,Z) %orbits

%Mark q+, q-
plot(q_p(1), q_p(2),'.', 'MarkerSize',10,'color','r');
plot(q_m(1), q_m(2), '.','MarkerSize',10,'color','g');

%Plot impact surfaces S+, S-
plot(q1_range, -2*q1_range(1,:) + 2*pi)
plot(q1_range, -2*q1_range)


figure(VHC_plot);
%Plot rough curve
% plot(q_rough(1,:), q_rough(2,:), 'o');
hold on


%Plot q_a, q_b
plot(qa(1), qa(2),'.', 'MarkerSize',10,'color','k');
plot(qb(1), qb(2), '.','MarkerSize',10,'color','k');

ax1 = gca;
VHC_plot_final_search = figure();
ax2 = copyobj(ax1,VHC_plot_final_search);
title('Searching for final curve');


%Plot the new curve
figure(VHC_plot);
theta_final = linspace(theta_start,theta_end,1000);
q_testing = ppval(sigma,theta_final);
plot(q_testing(1,:), q_testing(2,:), 'k'); %Check that cuve matches



%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               CHECKING LIMIT CYCLE CONDITIONS             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
syms q1 q2 q3 q4 real
syms q1dot q2dot q1d_dot q2d_dot
syms theta

q = [q1; q2];
qdot = [q1dot; q2dot];
qd_dot = [q1d_dot; q2d_dot];


%Physical parameters
m1_n = m(1);
m2_n = m(3);
mH_n = m(2);


lc1_n = l(1);
l1_n = l(2);
lc2_n = l(3);
l2_n = l(4);

g =9.8;


%Centers of mass
rc1 = lc1*[cos(q1); sin(q1)];
rH = l1*[cos(q1);sin(q1)];
rc2 = rH + lc2*[cos(q1+q2); sin(q1+q2)];

%Angular velocities
w_1_0 = [0;0;1]*(q1dot);
w_2_0 = [0;0;1]*(q1dot+q2dot);


rc1_dot = simplify(jacobian(rc1,q)*qdot);
rc2_dot = simplify(jacobian(rc2,q)*qdot);


%% Calculate the matrices

% Gravity vector
Ua = m1_n*g*rc1(2);
Ub = m2_n*g*rc2(2);

U = Ua + Ub;


jac_P = jacobian(U,q).';
jac_P = simplify(jac_P);

% inertia matrix
Ta = 0.5*m1_n*(transpose(rc1_dot)*rc1_dot);
Tb = 0.5*m2_n*(transpose(rc2_dot)*rc2_dot);

T = Ta + Tb;

D = simplify(jacobian(T,qdot).');
D = simplify(jacobian(D,qdot));


%Christoffel coefficients
syms C real
syms Q real
n = size(q,1);
fprintf('Computing Christoffel coefficients (GROSS >:( )\n')

for K=1:n
  for j=1:n
    C(K,j)=0;
    for i=1:n
      %Q(i,j,k) = c_ijk
      Q(i,j,K) = 1/2*(diff(D(K,j),q(i)) + ...
			 diff(D(K,i),q(j)) - ...
			 diff(D(i,j),q(K)));
      Q(i,j,K) = simplify(Q(i,j,K));
      C(K,j)=C(K,j) + Q(i,j,K)*qdot(i);
    end
  end
end


%% Compute Psi1, Psi2 terms

sigma_prime_sym = sym('sigma_prime',[2 1]);
sigma_d_prime_sym = sym('sigma_d_prime',[2 1]);

delta = B_p*D*sigma_prime_sym;

fprintf('Computing Psi1 and Psi2\n')
Psi1 = -B_p*jac_P/delta;

term1 = B_p*D*sigma_d_prime_sym;
term2 = 0;
for i=1:n
    Qi = Q(:,:,i);
    term2 = term2+B_p(i)*transpose(sigma_prime_sym)*Qi*sigma_prime_sym;

end

Psi2 = -(1/delta)*(term1 + term2);
%%

calc_Psi1_numeric = matlabFunction(Psi1,'File','Psi1_func');
calc_Psi2_numeric = matlabFunction(Psi2,'File','Psi2_func');


%% Sub in values for sigma = q, sigma_prime, sigma_d_prime

sigma_prime = fnder(sigma,1);
sigma_d_prime = fnder(sigma,2);

sigma_val = ppval(sigma,theta_final);
sigma_prime_val = ppval(sigma_prime,theta_final);
sigma_d_prime_val = ppval(sigma_d_prime,theta_final);


%% Calculate deltaz
theta_plus = theta_start;
theta_minus = theta_end;

sigma_prime_plus = ppval(sigma_prime,theta_plus);
sigma_prime_minus = ppval(sigma_prime,theta_minus);
deltaz = (1/(norm(sigma_prime_plus))^2)*(transpose(sigma_prime_plus)*delta_qsdot*sigma_prime_minus)


%%
M_0 = 1;
V_0 = 0;

x0 = [M_0;V_0];

[tt,x]=ode45(@(tt,x)M_V_numeric_sys(tt,x,calc_Psi1_numeric,calc_Psi2_numeric, sigma, sigma_prime, sigma_d_prime),theta_final,x0);


%%
M_vals = x(:,1);
V_vals = x(:,2);

M_plus = M_vals(1);
M_minus = M_vals(end);

V_plus = V_vals(1);
V_max = max(V_vals);
V_minus = V_vals(end);


disp(['M_minus: ',num2str(M_minus)])
disp(['V_minus: ',num2str(V_minus)])
disp(['V_max: ',num2str(V_max)])


%% Existence test; want this less than 0

temp_val = (M_plus/M_minus)*deltaz^2;
existence_test = (temp_val/(1-temp_val))*V_minus + V_max

%% Stability check; want this to be between 0 and 1
stability_test = deltaz^2*(M_plus/M_minus)



%%
figure
plot(theta_final,V_vals)
hold on
title('V and M values')
plot(theta_final,M_vals)

legend('V_vals', 'M_vals');
xlabel('theta');


%%
%% Checking regularity
% %Is the VHC transversal everywhere to inv(D)*B? Check that sigma prime is
% %never tangent to inv(D)*B. If it is ever tangent, the matrix will be
% %singular; i.e., if the curve crosses 0, then the VHC is not regular



for i=1:size(theta_final,2)
    temp = inv(calc_D(l1,lc1,lc2,m1,m2,sigma_val(2,i)))*B;
    detvals(i) = det([sigma_prime_val(:,i) temp]);

end
figure
plot(theta_final,detvals) %For length of the curve, plot det check
hold on
title('Regularity check');
xlabel('theta');
ylabel('detvals');

% return
%% Reduced dynamics

%Run reduced dynamics
tspan = 0:0.0001:5;
theta_0 = theta_start;

%fixed point
theta_d_minus = sqrt((-2*V_minus)/(M_minus-M_plus*deltaz^2));
theta_d_plus = theta_d_minus*deltaz;
% x0 = [theta_0; theta_d_plus];
%
%
%
% red_dyn_options = odeset('Events',@(t_sol,x_sol)theta_impact_event(t_sol,x_sol,theta_end), 'RelTol', 1e-15, 'AbsTol', 1e-10);
% [t_sol,x_sol,t_e,x_e]=ode45(@(t_sol,x_sol)reduced_dyn(t_sol,x_sol,calc_Psi1_numeric,calc_Psi2_numeric,sigma,sigma_prime, sigma_d_prime),tspan,x0,red_dyn_options);
%
% %x_sol gives theta, theta_dot
% x_sol(end,2) - theta_d_minus

%% Check total energy along solution

% M_0_test = 1;
% V_0_test = 0;
%
% x0_test = [M_0_test;V_0_test];
% thetaspan_test = x_sol(:,1);
%
%
%
% % [t2,x2]=ode45(@(t2,x2)M_V_numeric_sys(t2,x2,calc_Psi1_numeric,calc_Psi2_numeric, sigma, sigma_prime, sigma_d_prime),thetaspan_test,x0_test);
% % M_vals_test = x2(:,1);
% % V_vals_test = x2(:,2);
% % K_vals_test = zeros(size(V_vals_test));
% %
% % energy_vec = zeros(size(V_vals_test));
% % for k=1:size(V_vals_test)
% %     K_val = 0.5*M_vals_test(k)*x_sol(k,2)^2;
% %     K_vals_test(k) = K_val;
% %     energy_vec(k) = K_val + V_vals_test(k);
% % end
% %
% %
% % figure
% % plot(thetaspan_test,V_vals_test)
% % hold on
% % title('V and K values')
% % plot(thetaspan_test,K_vals_test)
% % plot(thetaspan_test,energy_vec);
% % legend('V_vals', 'K_vals','E');
% %



%%
% q1_range = 0:0.05:1.3;
% q2_range =0:0.01:10;
%
% q1_range = linspace(0,1.3,100);
% q2_range =linspace(0,20,100);
%
%
% q1_range = linspace(0,1.3,100);
% q2_range =linspace(-5,20,100);
% [X1,X2] = meshgrid(q1_range,q2_range);
%
% sigma_grid = ppval(sigma,X1);
% sigma_prime_grid = ppval(sigma_prime,X1);
% sigma_d_prime_grid = ppval(sigma_d_prime,X1);
%
%
% r = zeros(size(X1));
% z = zeros(size(X2));
%
% %Dinv*B
% for i=1:size(X2,1)
%     for j=1:size(X2,2)
%         theta = X1(i,j);
%         theta_dot = X2(i,j);
%         a = sigma_grid(:,i,j);
%         q1 = a(1);
%         q2 = a(2);
% %         Psi1 = Psi1_func(q1,q2,sigma_prime_val(1),sigma_prime_val(2));
% %         Psi2 = Psi2_func(q2,sigma_prime_val(1),sigma_prime_val(2),sigma_d_prime_val(1),sigma_d_prime_val(2));
%         sigma_prime_val_temp = sigma_prime_grid(:,i,j);
%         sigma_d_prime_val_temp = sigma_d_prime_grid(:,i,j);
%
%         Psi1 = Psi1_func(q1,q2,sigma_prime_val_temp(1),sigma_prime_val_temp(2));
%         Psi2 = Psi2_func(q2,sigma_prime_val_temp(1),sigma_prime_val_temp(2),sigma_d_prime_val_temp(1),sigma_d_prime_val_temp(2));
%         theta_ddot = Psi1 + Psi2*theta_dot^2;
%
%         r(i,j) = theta_dot; %thetadot
%         z(i,j) = theta_ddot; %thetaddot
%     end
% end
% figure
% hold on
% xlabel('theta');
% ylabel('theta_dot');
% streamslice(X1,X2,r,z,10) %orbits
% plot(x_sol(:,1),x_sol(:,2),'r');
% hold on


%% Running the reduced dynamics with impact
%
% %Run reduced dynamics
%
% tmax = 20;
% t_step = 0.0001;
% tspan = 0:t_step:tmax;
% theta_0 = theta_start;
%
% %fixed point
% theta_d_minus = sqrt((-2*V_minus)/(M_minus-M_plus*deltaz^2));
% theta_d_plus = theta_d_minus*deltaz;
% % x0 = [theta_0; theta_d_plus];
%
%
% epsilon = 0;
% x0 = [theta_0; theta_d_plus+epsilon];
%
%
% %Holds total solutions
% x_sol_tot = zeros(size(tspan,2),2);
% t_sol_tot = zeros(size(tspan,2),1);
%
% %Theta offset set to 0 in the loop to always keep from 0 to 1
% theta_offset = 0;
%
% %First step
% red_dyn_options = odeset('Events',@(t_sol,x_sol)theta_impact_event(t_sol,x_sol,theta_end), 'RelTol', 1e-15, 'AbsTol', 1e-10);
% [t_sol,x_sol,t_e,x_e]=ode45(@(t_sol,x_sol)reduced_dyn(t_sol,x_sol,calc_Psi1_numeric,calc_Psi2_numeric,sigma,sigma_prime, sigma_d_prime),tspan,x0,red_dyn_options);
%
% theta_d_minus_err = x_sol(end,2) - theta_d_minus; %how far from fixed point
% theta_d_plus_err = deltaz*x_sol(end,2) - theta_d_plus
%
% t_stop_index = size(t_sol,1) -1; %Index of last calculated time (-1 to remove t_minus)
% step_time = size(t,1)-1; %Time for the step (-1 to remove t_minus)
% x_sol_tot(1:t_stop_index,:) = x_sol(1:end-1,:); %Store first step solution
% t_sol_tot(1:t_stop_index,:) = t_sol(1:end-1,:);
%
%
% while isempty(t_e) ~= 1
%     %Impact map - new initial conditoins
%     t_e = t_e(size(t_e,1));
%     x_e = x_e(size(x_e,1),:);
%     tspan_new = t_e:t_step:tmax;
%     theta_offset = theta_offset+1;
%
%     x0_new = [0; deltaz*x_e(2)];
%
%     [t_sol_new,x_sol_new,t_e,x_e]=ode45(@(t_sol_new,x_sol_new)reduced_dyn(t_sol_new,x_sol_new,calc_Psi1_numeric,calc_Psi2_numeric,sigma,sigma_prime, sigma_d_prime),tspan_new,x0_new,red_dyn_options);
%
%     %Save new values to vector
%     step_time = size(t_sol_new,1)-1; %Time for current step
%     theta_offset_vec = [ones(size(t_sol_new,1)-1,1)*theta_offset zeros(size(t_sol_new,1)-1,1)];
%     x_sol_tot(t_stop_index+1:t_stop_index+step_time,:)=x_sol_new(1:end-1,:)+theta_offset_vec;
%     t_sol_tot(t_stop_index+1:t_stop_index+step_time,:)=t_sol_new(1:end-1,:);
%
%     %Update stop index
%     t_stop_index = t_stop_index+step_time;
%
%     theta_d_minus_err = x_sol_new(end,2) - theta_d_minus;
%     theta_d_plus_err = deltaz*x_sol_new(end,2) - theta_d_plus
%
% end
%%
% multiple_steps = figure;
% hold on
% xlabel('theta');
% ylabel('theta_dot');
% plot(x_sol_tot(:,1),x_sol_tot(:,2),'r');

%%
% sigma1_inv = spline(q_testing(1,:), theta_final);
% sigma2 = spline(theta_final, q_testing(2,:));
% test = linspace(1.963,1.1781,200);
% zz = ppval(sigma1_inv,test);
% figure;
% plot(test,zz);


theta_f = linspace(theta_rough(1),theta_rough(end),800);
q_f = ppval(sigma,theta_f);
figure
hold on
plot(q_f(1,:), q_f(2,:));


g_func = spline(q_f(1,:),q_f(2,:)); %q2 as function of q1
yy = linspace(q_f(1,1), q_f(1,end));
zz = ppval(g_func, yy);
plot(yy, zz, '.');

aa = linspace(q_p(1),q_m(1),800);
bb = ppval(g_func,aa);
plot(aa,bb,'g');

% save('h_vars.mat', 'sigma1_inv', 'sigma2');
save('h_vars.mat', 'g_func', 'sigma','theta_d_plus', 'beta');
save('h_extra_vars.mat','theta_start','theta_end', 'q_p','q_m');

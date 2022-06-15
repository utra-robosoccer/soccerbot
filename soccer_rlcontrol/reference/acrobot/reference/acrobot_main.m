%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ver. Sept 21, 2019                                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Main file for computing all necessary EOM, controller, and impact map expressions
clc; clear;
close all;
%% Set to 1 if need to recompute symbolic expressions; otherwise load files
run_from_scratch = 1; %1 = run symbolic code; 0 = load symbolic expressions

if run_from_scratch ==1
    acrobot_symbolic
end+

load('acrobot_syms.mat');
load('h_vars.mat');
load('h_extra_vars.mat');


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          PART 4: SIMULATION                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %Parts 1-3 in symbolic code
%% Set up parameters for 4 DOF biped simulation

tmax = 6; %Max simulation time
t_step = 0.005; %Simulation time step
tspan = 0:t_step:tmax;

%Physical parameters
% NOTE: The current VHC works for these parameters; it will most likely not
% work with different parameters
load_acrobot_params

%Controller parameters
B = [0;1]; %2x1; actuate the hip

%Control parameters for feedback linearization
gamma = 0.05;
Kp = (1/gamma)^2;
Kd = 2/gamma;

%Initial conditions
q_dot_0 = ppval(fnder(sigma,1),0)*7.2295; %Values when on the limit cycle
x0 = [(pi+beta)/2 - 0.05; pi-beta; q_dot_0(1)-1; q_dot_0(2)+1];  %Currently set to start off the configuration manifold


%% Vectors to hold final results
xtot = zeros(size(tspan,2),size(x0,1)); %Holds overall solution with multiple steps
ttot = zeros(size(tspan,2),1); %Holds overall solution time
offset = zeros(size(tspan,2),2); %Holds offset for stance foot
x0_new = 0; %Updates after each impact to store new initial conditions


%% First step
%Stops simulation when foot impacts the ground
options = odeset('Events',@(t,x)impact_event(t,x,l), 'RelTol', 1e-9, 'AbsTol', 1e-9);

%% Run the first step
[t,x,te,xe]=ode45(@(t,x)acrobot(t,x,l,m,B,calc_D,calc_b1, beta, Kp,Kd,g_func),tspan,x0,options);


%% Store the first step
t_stop_index = size(t,1) -1; %Index of last calculated time (-1 to remove x_impact_minus)
step_time = size(t,1)-1; %Time for the step (-1 to remove x_impact_minus)
xtot(1:t_stop_index,:) = x(1:end-1,:); %Store first step solution
ttot(1:t_stop_index,:) = t(1:end-1,:);

next_offset = [0,0]; %holds position of swing leg


%% Simulate remaining steps

while (isempty(te) ~= 1)&&((tmax-te(size(te,1)))>t_step)
    %Take only last event (ending step)
    te = te(size(te,1));
    xe = xe(size(xe,1),:);
    tspan_new = te:t_step:tmax;

    %Pre-impact configuration
    q1 = xe(1);
    q2 = xe(2);

    rH = l1*[cos(q1);sin(q1)]; %Hip position
    P_heel2 = rH + l2*[cos(q1+q2); sin(q1+q2)]; %Swing foot position


    %Swing foot ahead of stance foot; end of step impact occurs
    %Calculate new initial conditions using impact map
    x0_new = acrobot_impact_map(xe,l,m,calc_De,calc_E2,calc_dUpsilon_e_dqs);
    %Wrap to 2*pi
    x0_new(1:2) = wrapTo2Pi(x0_new(1:2));

    %Calculate offset for next step (position of swing leg at end of
    %step is posn of stance leg at start of next step)
    next_offset = next_offset+ P_heel2';

    %Restart simulation with new initial conditions
    [t_new,x_new,te,xe]=ode45(@(t_new,x_new)acrobot(t_new,x_new,l,m,B,calc_D,calc_b1, beta, Kp,Kd,g_func),tspan_new,x0_new,options);

    %Save new values to vector
    step_time = size(t_new,1)-1; %Time for current step
    xtot(t_stop_index+1:t_stop_index+step_time,:)=x_new(1:end-1,:);
    ttot(t_stop_index+1:t_stop_index+step_time,:)=t_new(1:end-1,:);

    %Update stop index
    offset(t_stop_index+1:t_stop_index+step_time,:) = ones(step_time,1)*next_offset;
    t_stop_index = t_stop_index+step_time;
end

xtot = xtot(1:t_stop_index,:);
ttot = ttot(1:t_stop_index,:);
offset = offset(1:t_stop_index,:);

%% Check that satisfying the constraint
q2_desired = ppval(g_func,xtot(:,1));
VHC_check = figure;
hold on
xlabel('simulation time');
ylabel('q2_{desired} - q2');
plot(ttot, (q2_desired - xtot(:,2)));


%% Animation
scrolling = 1; %Set to 1 for plot window to scroll with acrobot
animate_acrobot_VHC(ttot,xtot,l,m,offset,scrolling,calc_D,B,sigma,q_m,q_p,theta_start,theta_end)

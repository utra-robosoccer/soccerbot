function[] = plot_acrobot(x,l,phi,plot_constraints,h)
%Plot acrobot for a specific configuration(s)
%x = vector of configurations
%plot_constraints; 0 = don't plot, 1 = plot

lc1 = l(1);
l1 = l(2);
lc2 = l(3);
l2 = l(4);


q1 = x(:,1);
q2 = x(:,2);

fig = figure;
hold on


%Plot ground
Xslope_plot = linspace(-10,10,100);
Yslope_plot = linspace(-10,10,100)*tan(phi);
plot(Xslope_plot,Yslope_plot)

axis_vec = [-0.6 0.8 -0.5 1.3];
axis(axis_vec);

%Rotation matrix for slope
R_1_0 = [cos(phi) -sin(phi); sin(phi) cos(phi)];


%Centers of mass
rc1 = lc1*[cos(q1), sin(q1)]*R_1_0';
rH = l1*[cos(q1), sin(q1)]*R_1_0'; %Hip posn
rc2 = rH + lc2*[cos(q1+q2), sin(q1+q2)]*R_1_0';

P_heel2 = rH + l2*[cos(q1+q2), sin(q1+q2)]*R_1_0';
P_heel1 = zeros(size(P_heel2));

for i=1:size(x,1)


    %Plot
    p1 = plot(rc1(i,1), rc1(i,2), '.', 'markersize',20,'color','b'); %Stance leg mass
    p2 = plot(rc2(i,1), rc2(i,2), '.', 'markersize',20,'color','b'); %swing leg mass


    r1 = line([P_heel1(i,1),rH(i,1)],[P_heel1(i,2),rH(i,2)]); %heel1 to hip
    r2 = line([rH(i,1),P_heel2(i,1)],[rH(i,2),P_heel2(i,2)] ); %hip to heel2

    r1.Color = 'blue';
    r2.Color = 'black';
end

%% CONSTRAINT
if plot_constraints == 1
    syms q1 q2
    q2_symb = solve(h,q2);
    %calculate the trajectory of swing foot
    q1_valrange = linspace(pi,0,100);
    q2_calcvals = subs(q2_symb,q1_valrange);
    rH_calcvals = l1*[cos(q1_valrange)', sin(q1_valrange)']*R_1_0';
    P_heel2_calcvals = rH_calcvals + l2*[cos(q1_valrange+q2_calcvals)', sin(q1_valrange+q2_calcvals)']*R_1_0';

    plot(P_heel2_calcvals(:,1),P_heel2_calcvals(:,2));
end

end

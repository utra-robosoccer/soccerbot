function[] = animate_acrobot_VHC(t,x,l,m,offset,scrolling,calc_D,B,sigma,q_m,q_p,theta_start,theta_end)
%Animation for rocker ankle biped walker showing the constraints
% offset = offset*0;
lc1 = l(1);
l1 = l(2);
lc2 = l(3);
l2 = l(4);

m1 = m(1);
mH = m(2);
m2 = m(3);


q1 = x(:,1);
q2 = x(:,2);

fig = figure;
set(fig, 'Position',  [100, 100, 1500, 700]);
figure(fig)
subplot(1,2,1);
hold on
subplot(1,2,2);
hold on


% set(gcf, 'Position', [200, 200, 600, 600])

%PLOT ROBOT
subplot(1,2,1)
Xslope_plot = linspace(-10,10,100);
Yslope_plot = 0*Xslope_plot;
plot(Xslope_plot,Yslope_plot)

% axis([-2 2 -2 2]);
axis_vec = [-0.6 0.8 -0.5 0.9];
axis square
axis_shift = axis_vec(2)-axis_vec(1)-0.2;
axis(axis_vec);

%Heel of stance foot
P_heel1 = offset;

%Centers of mass
rc1 = lc1*[cos(q1), sin(q1)]+offset;
rH = l1*[cos(q1), sin(q1)] + offset; %Hip posn
rc2 = rH + lc2*[cos(q1+q2), sin(q1+q2)];

P_heel2 = rH + l2*[cos(q1+q2), sin(q1+q2)];

p1 = plot(rc1(1,1), rc1(1,2), '.', 'markersize',20,'color','b'); %Stance leg mass
p2 = plot(rc2(1,1), rc2(1,2), '.', 'markersize',20,'color','b'); %swing leg mass
r1 = line([P_heel1(1,1),rH(1,1)],[P_heel1(1,2),rH(1,2)]); %heel1 to hip
r2 = line([rH(1,1),P_heel2(1,1)],[rH(1,2),P_heel2(1,2)] ); %hip to heel2

r1.Color = 'blue';
r2.Color = 'black';



%PLOT VECTOR FIELD INV(D)*B
subplot(1,2,2)
q1_range = 0/16:0.05:pi;
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

axis square
streamslice(X1,X2,R,Z) %orbits

%Mark q+, q-
plot(q_p(1), q_p(2),'.', 'MarkerSize',10,'color','r');
plot(q_m(1), q_m(2), '.','MarkerSize',10,'color','g');

%Plot impact surfaces S+, S-
plot(q1_range, -2*q1_range(1,:) + 2*pi)
plot(q1_range, -2*q1_range)
theta_final = linspace(theta_start,theta_end,800);
q_testing = ppval(sigma,theta_final);
plot(q_testing(1,:), q_testing(2,:), 'k'); %Check that cuve matches


p_theta = plot(q1(1), q2(1), '.', 'markersize',20,'color','m');

timedisp = text(-0.8,0.5,sprintf('time: %f', t(1))); %Display current time


% %Save to video
% vobj = VideoWriter('test2', 'MPEG-4');
% vobj.FrameRate=25;
% vobj.Quality=75
% open(vobj)

figure(fig)
for i=2:length(t)
    timerVal = tic;

    set(p1, 'xdata', rc1(i,1), 'ydata', rc1(i,2)); %Stance shank mass
    set(p2, 'xdata', rc2(i,1), 'ydata', rc2(i,2)); %Stance thigh mass
%     set(pH, 'xdata', rH(i,1), 'ydata', rH(i,2)); %Hip mass

    set(r1, 'xdata', [P_heel1(i,1),rH(i,1)], 'ydata', [P_heel1(i,2),rH(i,2)]); %Stance leg
    set(r2, 'xdata', [rH(i,1),P_heel2(i,1)], 'ydata', [rH(i,2),P_heel2(i,2)]); %Swing leg

    set(timedisp, 'String', sprintf('time: %f', t(i))); %Display current time


    %Update axes
    figure(fig)
    subplot(1,2,1)
    if ((scrolling==1)&&(P_heel2(i,1)+0.1) > axis_vec(2))
        axis_vec = axis_vec+[axis_shift axis_shift 0 0];
        axis(axis_vec);
    end
    figure(fig)
    subplot(1,2,2);
    set(p_theta, 'xdata', q1(i), 'ydata', q2(i)); %Stance leg



%     drawnow
%     pause(0.1)
%     fig.GraphicsSmoothing='off'
%     %For generating gif
%     frame = getframe(fig);
% %     im{i} = frame2im(frame);
%     writeVideo(vobj,frame);


    pause(0.01) %Change framerate
end
% close(vobj)

end

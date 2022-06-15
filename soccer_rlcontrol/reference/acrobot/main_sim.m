close all; clc;

robot = acrobot.acrobot_control(false);
robot.resetRobot();

tmax = 10;       % Max simulation time
tstep = 0.01;  % Simulation time step
t = 0;

% Simulate robot falling on the ground
fig = figure;
set(fig, 'Position',  [100, 100, 1500, 700]);
options = odeset('Events',@(t,x)robot.dist_to_floor(t,x), 'RelTol', 1e-3, 'AbsTol', 1e-6);
rate = rateControl(1/tstep);

% Simulate Robot Walking
while (t < tmax)
    tic;

    % Calculate the value for tau at the point
    tau = robot.getTau(robot.x);

    t1 = toc;

    % Search for foot placement when close to floor
    t_next = floor((t + tstep + 1e-9)/tstep)*tstep;
    [t_anim, x_anim, te, xe, ie] = ode45(@(t, x) robot.physics_step(t, x, tau), [t t_next], robot.x, options);

    if (ie)
        robot.impact_foot(xe);
        t = te;
    else
        robot.x = x_anim(end,:)';
        t = t_next;
    end

    % End Conditions
    if (robot.x(2) > pi - robot.angle_limit || robot.x(2) < -pi + robot.angle_limit || robot.x(1) > pi || robot.x(1) < 0)
        disp("Robot Impacted With Itself");
        break
    end

    t2 = toc;

    set(0, 'CurrentFigure', fig)
    robot.show(t);

    t3 = toc;

    disp(strcat("Get Tau Time: ", num2str(t1), " Step Time: ", num2str(t2), " Display Time: ", num2str(t3)));
    waitfor(rate);
end

robotParameters;
if exist('robot', 'var')
    robot.resetRobot();
else
    robot = acrobot.acrobot_control(false);
end

slowdown = 1.0;
tstep = 0.035 / slowdown;  % Time step
rate = rateControl(1/tstep);

close all;
fig = figure;
set(fig, 'Position',  [100, 100, 1500, 700]);
listing = dir('data/tests/');
load(strcat('data/tests/',listing(end).name));

options = odeset('Events',@(t,x)robot.dist_to_floor(t,x), 'RelTol', 1e-9, 'AbsTol', 1e-9);

for t = ts.Time'
    robot.x = ts.getsampleusingtime(t).Data(1:4);
    is_collision = ts.getsampleusingtime(t).Data(5);
    tau = robot.getTau(robot.x);
%    [t_anim, x_anim, te, xe, ie] = ode45(@(t, x) robot.physics_step(t, x, tau), [t t+tstep], robot.x, options);
    if (is_collision)
        robot.impact_foot(robot.x);
        disp(strcat("Expected: ", num2str(robot.x')));
    end
    fprintf("t: %.3f\t X: %.3f %.3f %.3f %.3f\t tau %.3f\n", t, robot.x(1), robot.x(2), robot.x(3), robot.x(4), tau);
    robot.show(t);
    waitfor(rate);
end

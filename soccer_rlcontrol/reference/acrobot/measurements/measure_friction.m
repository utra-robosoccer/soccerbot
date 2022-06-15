clear; clc;
close all;
a = arduino('/dev/ttyUSB0','Nano3','Libraries',{'RotaryEncoder', 'I2C'});
encoder = rotaryEncoder(a,'D2','D3');

robot = acrobot.acrobot_control();
robot_t_control = acrobot.acrobot_torque_control();
%%
encoder.resetCount();

tau = 0.20;
tstop = 0.1;
tend = 0.5;
time = [];
pwms = [];
angles = [];
torques = [];

pwm = robot_t_control.getPWM(tau);

if (pwm < 0)
    writeDigitalPin(a, 'D6', 1);
    writeDigitalPin(a, 'D7', 0);
elseif (pwm >= 0)
    writeDigitalPin(a, 'D6', 0);
    writeDigitalPin(a, 'D7', 1);
end

delta_t = 0.01;
rate = rateControl(1/delta_t);
tstart = tic;
writePWMDutyCycle(a,'D9',abs(pwm));
stopped = 0;
t = 0;
while(t < tend)
    t = toc(tstart);
    tic
    % Measurement
    count = readCount(encoder);
    measuredAngle = (360.0 / 2797.0) * count;

    angles = [angles, measuredAngle];
    time = [time, t];

    if t > tstop && ~stopped
        writePWMDutyCycle(a,'D9',0);
        stopped = 1;
    end

    if stopped
        torques = [torques, 0];
        pwms = [pwms, 0];
    else
        torques = [torques, tau];
        pwms = [pwms, pwm];
    end
    toc
    waitfor(rate);
end

%% Get predicted angle rate using the friction parameters
syms theta(t) I ks kv taum theta0 thetadot0 real
Dtheta = diff(theta);
ode = taum - kv * diff(theta, t) == I * diff(theta, t, 2);
cond1 = theta(0) == theta0;
cond2 = Dtheta(0) == thetadot0;
thetasolv = simplify(dsolve(ode, [cond1, cond2]));
thetasolvv = matlabFunction(thetasolv);
thetadotsolv = diff(thetasolv);
thetadotsolvv = matlabFunction(thetadotsolv);


kv = 0.01;
ks = 0.030;
I = 2.32e-3;
t1 = 0:0.01:tstop;
t2 = (tstop:0.01:tend) - tstop;
p1 = thetasolvv(I,kv,t1,tau,0,0);
v1 = thetadotsolvv(I,kv,t1,tau,0);
p2 = thetasolvv(I,kv,t2,-ks,p1(end),v1(end));

t = [t1 t2 + tstop];
p = [p1 p2];


hold on
plot(time, angles);
hold on;
plot(t, rad2deg(-p));
title('Measured Angle vs Time');
xlabel('Time (s)');
ylabel('Angle (degrees)');
grid minor;

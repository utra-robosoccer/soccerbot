clear; clc;
close all;
a = arduino('/dev/ttyUSB0','Nano3','Libraries',{'RotaryEncoder', 'I2C'});
encoder = rotaryEncoder(a,'D2','D3');

robot = acrobot.acrobot_control();
robot_t_control = acrobot.acrobot_torque_control();
%%
gain = pi/8;
speed = 10;
t_duration = 10;
delta_t = 0.035;
rate = rateControl(1/delta_t);
t_step = 0:delta_t:t_duration;
value = sin(t_step*speed) * gain;

gain = pi/16;
measuredAngle = 0;
frequency = 2;
cpr = 2797; % encoder resolution
encoder.resetCount();

t0 = clock;
lastTime = t0;
lastErr = 1;
errSum = 0;
angles = [];
desangles = [];
torques = [];
pwms = [];
time = [];

robot.kp = 0.8;
robot.kd = 150;
0.0;
robot.ki = 0.02;
robot.kb = 0.01;
robot.integral_saturation = 1;

t = 0;
t_last = 0;
pwm = 0;
tstart = tic;
i = 0;
while(t < 10)
    t = toc(tstart);
    dt = t - t_last;
    t_last = t;

    i = i + 1;
    desiredAngle = value(i);


    count = readCount(encoder);
    measuredAngle = deg2rad((360 / cpr) * count);

    angles = [angles, measuredAngle];
    desangles = [desangles, desiredAngle];
    time = [time, t];

    % PID
    error = measuredAngle - desiredAngle;
    derror = (error - lastErr) * dt;
    errSum = error + errSum;
    u = (robot.kp * error) + (robot.ki * errSum) + (robot.kd * derror);
    if (abs(errSum) > robot.integral_saturation)
        if (errSum > 0)
            errSum = errSum + (robot.integral_saturation - errSum) * robot.kb;
        else
            errSum = errSum + (-robot.integral_saturation - errSum) * robot.kb;
        end
    end

    torques = [torques, u];

    pwm_new = min(1,max(-1,u))
    if (pwm_new < 0 && pwm >= 0)
        writeDigitalPin(a, 'D6', 1);
        writeDigitalPin(a, 'D7', 0);
    elseif (pwm_new > 0 && pwm <= 0)
        writeDigitalPin(a, 'D6', 0);
        writeDigitalPin(a, 'D7', 1);
    end
    pwm = pwm_new;
    pwms = [pwms, pwm];
    writePWMDutyCycle(a,'D9',abs(pwm));

    lastErr = error;
    waitfor(rate);
end

writePWMDutyCycle(a,'D9',0.0);
%%
figure;
subplot(3,1,1)
plot(time, angles);
hold on;
plot(time, desangles);
title('Measured Angle vs Time');
xlabel('Time (s)');
ylabel('Angle (radians)');
grid minor;

subplot(3,1,2)
plot(time, torques);
title('Torque vs Time');
xlabel('Time (s)');
ylabel('Torque');
grid minor;

subplot(3,1,3)
plot(time, pwms);
title('PWM vs Time');
xlabel('Time (s)');
ylabel('Duty Cycle');
grid minor;

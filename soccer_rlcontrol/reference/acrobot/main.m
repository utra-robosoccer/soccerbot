%% Device Connection

try
    a.readVoltage("A0");

catch ex
    clear;
    robotParameters;
    try
        a = arduino('/dev/ttyACM0','MKR1000','BaudRate',1000000,'Libraries',{'RotaryEncoder', 'I2C'});
    catch
        a = arduino('/dev/ttyACM1','MKR1000','BaudRate',1000000,'Libraries',{'RotaryEncoder', 'I2C'});
    end

    BNO1 = device(a,'I2CAddress', '0x28');
    BNO2 = device(a,'I2CAddress', '0x29');
    encoder = rotaryEncoder(a, 'D0','D1', steps_per_rotation);
end


close all; clc;
writeRegister(BNO2,hex2dec('3F'), hex2dec('20'),'uint8');
writeRegister(BNO1,hex2dec('3F'), hex2dec('20'),'uint8');

pause(0.5);

writeRegister(BNO2,hex2dec('3D'),hex2dec('00'),'uint8');
writeRegister(BNO1,hex2dec('3D'),hex2dec('00'),'uint8');

pause(0.5);

writeRegister(BNO2,hex2dec('42'),hex2dec('00'),'uint8');
writeRegister(BNO1,hex2dec('42'),hex2dec('00'),'uint8');

pause(0.5);

writeRegister(BNO2,hex2dec('3D'),hex2dec('08'),'uint8');
writeRegister(BNO1,hex2dec('3D'),hex2dec('08'),'uint8');

pause(0.5);

writePWMDutyCycle(a,'D4',0);

robotParameters;
robot = acrobot.acrobot_control(false);
estimator = acrobot.acrobot_state_estimator();
torque_control = acrobot.acrobot_torque_control();
robot.actual_robot = 1;
robot.kp = 50;
robot.ki = 0;
robot.kd = 15;
robot.kb = 0.01;

tstep = 0.05;  % Time step
rate = rateControl(1/tstep);
rotmXYZ = eul2rotm([0 pi 0], 'XYZ');
test_state_estimation = 0;
setup_duration = 0.2;
if test_state_estimation
    fig = figure;
    set(fig, 'Position',  [100, 100, 1500, 700]);
    duration = 10;
else
    duration = 8;
end

estimator.sample_time = tstep;
estimator.setupImplPublic();
encoder.resetCount();
robot.resetRobot();

last_motor_step = encoder.readCount();
t = 0;
ts = timeseries('acrobot_data');

[~, ~, pos] = read_data(BNO2);
if (pos(2) == 0)
    disp("BNO02 not working");
    close all;
    return;
elseif(pos(2) > 0)
    BN02_offset = pos(2) - pi;
else
    BN02_offset = pos(2) + pi;
end

[~, ~, pos] = read_data(BNO1);
if (pos(2) == 0)
    disp("BNO01 not working");
    close all;
    return;
elseif (pos(2) > 0)
    BN01_offset = -pos(2);
else
    BN01_offset = pos(2);
end

% Auto calibrate

pwm = 0;
writeDigitalPin(a, 'D2', 0);
writeDigitalPin(a, 'D3', 1);

try
    while (t < duration)
        tstart = tic;
        motor_step = -encoder.readCount();

        % State Estimation
        try
            if mod(robot.step_count, 2) == 0
                [acc, ~, pos] = read_data(BNO2);
                pos = wrapToPi(-pos + BN02_offset);
            else
                [acc, ~, pos] = read_data(BNO1);
                pos = wrapToPi(pos + BN01_offset);
            end
            [robot.x, collision] = estimator.stepImplPublic(robot.step_count, pos, acc, motor_step);

        catch ex
            if (contains(ex.message, "I2C"))
                disp("IMU Data lost");
                continue
            else
                rethrow(ex);
            end
        end


        if (collision)
            robot.step_count = robot.step_count + 1;
        end

        % Control Code
        tau = robot.getTau(robot.x);
        if mod(robot.step_count, 2) == 0
            tau = -tau;
        end

        % End Conditions
        if (robot.x(2) > pi - robot.angle_limit || robot.x(2) < -pi + robot.angle_limit || robot.x(1) > pi || robot.x(1) < 0)
            disp(strcat("Robot Impacted With Itself, angles: x1: ", num2str(robot.x(1)), " x2: ", num2str(robot.x(2))));
            break
        end
        % Motor Output
        pwm_new = torque_control.getPWM(tau(2));

        if (pwm_new < 0)
            writeDigitalPin(a, 'D2', 1);
            writeDigitalPin(a, 'D3', 0);
        elseif (pwm_new > 0)
            writeDigitalPin(a, 'D2', 0);
            writeDigitalPin(a, 'D3', 1);
        end
        pwm = pwm_new;

        if test_state_estimation
            robot.show(t);
            writePWMDutyCycle(a,'D4',0);
        elseif (t > setup_duration)
            writePWMDutyCycle(a,'D4',abs(pwm));
        end

        % Display the robot
        ts = ts.addsample('Data',[robot.x; collision],'Time',t);
        tend = toc(tstart);
        fprintf("Time: %.3f\t Elapsed Time: %.3f\t x1: %.3f\t x2: %.3f\t tau:%.3f\t pwm: %.3f\n", t, tend, robot.x(1), robot.x(2), tau(2), pwm);

        % Read next step
        t = t + tend;
    end
catch ex
    disp(ex)
    writeDigitalPin(a, 'D2', 0);
    writeDigitalPin(a, 'D3', 1);
    writePWMDutyCycle(a,'D4',0);
end
writeDigitalPin(a, 'D2', 0);
writeDigitalPin(a, 'D3', 1);
writePWMDutyCycle(a,'D4',0);

filename = sprintf('data/tests/test_%s', datestr(now,'mm-dd-yyyy HH-MM'));
save(filename, 'ts')
%%
writeDigitalPin(a, 'D2', 0);
writeDigitalPin(a, 'D3', 1);
writePWMDutyCycle(a,'D4',0);
%%
function [acc, gyro, pos] = read_data(BNO)
%     x = double(readRegister(BNO,hex2dec('8'),'int16')) / 100.0;
%     y = double(readRegister(BNO,hex2dec('A'),'int16')) / 100.0;
%     z = double(readRegister(BNO,hex2dec('C'),'int16')) / 100.0;
    acc = [0 0 0];

%     x = double(readRegister(BNO,hex2dec('14'),'int16')) / 16.0;
%     y = double(readRegister(BNO,hex2dec('16'),'int16')) / 16.0;
%     z = double(readRegister(BNO,hex2dec('18'),'int16')) / 16.0;
%     t_gyro = [x y z];
%     gyro = convangvel(t_gyro, 'deg/s' ,'rad/s');
    gyro = [0 0 0];

    deg_pitch = double(readRegister(BNO,hex2dec('1E'),'int16')) / 16.0; % Reads bits 15:8 from register 23
    rad_pitch = deg2rad(deg_pitch);
    pos = [0 rad_pitch 0];
end

%%

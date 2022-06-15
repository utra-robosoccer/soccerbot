%% Arduino Rotary Encoder input
clear; clc;
load ('data/speeds.mat')
a = arduino('/dev/ttyUSB1','Nano3','BaudRate',115200,'Libraries',{'I2C', 'RotaryEncoder'});
steps_per_rotation = 2797;
encoder = rotaryEncoder(a,'D2','D3',steps_per_rotation);

%% Count Encoder
pwm_step = 0.02;
trials = 50;
pwms = -1:pwm_step:1;
tstep = 0.01;
time_duration = 0:tstep:0.4;

for j = 47:1:length(pwms)
    for k = 1:1:trials
        if (pwms(j) > 0)
            writeDigitalPin(a, 'D6', 1);
            writeDigitalPin(a, 'D7', 0);
        else
            writeDigitalPin(a, 'D6', 0);
            writeDigitalPin(a, 'D7', 1);
        end
        % Stop the movement
        writePWMDutyCycle(a,'D9',0);
        pause(1.6);
        encoder.resetCount();

        % Record the speed increase
        rate = rateControl(1/tstep);
        position = zeros(1, length(duration));
        power = min(abs(pwms(j)),1);
        writePWMDutyCycle(a,'D9',power);
        for i = 1:1:length(time_duration)
            position(i) = readCount(encoder) / steps_per_rotation;
            pwm_values{j,k} = position;
            waitfor(rate);
        end
        disp(strcat('PWM: ', num2str(pwms(j)), ' Trial: ', num2str(k)));
    end
end
save ('data/speeds.mat')
writePWMDutyCycle(a,'D9',0);
%%
load ('data/speeds.mat')

hold off;
for j = 1:10:1
    p_avg = zeros(1,length(time_duration));
    v_avg = zeros(1,length(time_duration));
    a_avg = zeros(1,length(time_duration));

    for k = 1:1:trials
        p_avg = p_avg + pwm_values{j,k};

        p = spline(time_duration, pwm_values{j,k});
        v_avg = v_avg + ppval(fnder(p,1), time_duration);
        a_avg = a_avg + ppval(fnder(p,2), time_duration);
    end
    p_avg = p_avg / trials;
    v_avg = v_avg / trials;
    a_avg = a_avg / trials;


    subplot(3,1,1);
    for k = 1:1:trials
        plot(time_duration, pwm_values{j,k}, 'Color', [0,0,1,0.1])
        hold on;
        grid minor;
        xlabel('Time t');
        ylabel('Position r/s');
        title('Position vs time graph');
    end
    plot(time_duration, p_avg, 'Color', 'black')
    hold on;
    grid minor;
    xlabel('Time t');
    ylabel('Position r');
    title('Position vs time graph');

    subplot(3,1,2);
    for k = 1:1:trials
        p = spline(time_duration, pwm_values{j,k});
        plot(time_duration, ppval(fnder(p,1), time_duration), 'Color', [0,0,1,0.1])
        hold on;
    end
    plot(time_duration, v_avg, 'Color', 'black')
    hold on;
    grid minor;
    xlabel('Time t');
    ylabel('Velocity r/s');
    title('Speed vs time graph');

    subplot(3,1,3);
    for k = 1:1:trials
        p = spline(time_duration, pwm_values{j,k});
        plot(time_duration, ppval(fnder(p,2), time_duration), 'Color', [0,0,1,0.1])
        hold on;
    end
    plot(time_duration, smooth(a_avg), 'Color', 'black')
    hold on;
    grid minor;
    xlabel('Time t');
    ylabel('Acceleration r/s^2');
    title('Acceleration vs time graph');
end

%%
load ('data/speeds.mat')

hold off;
amount = 1:20;
acc_start = zeros(1,length(pwms));
for j = 1:1:length(pwms)
    a_avg = zeros(1,length(time_duration));

    for k = 1:1:trials
        p = spline(time_duration, pwm_values{j,k});
        a_avg = a_avg + ppval(fnder(p,2), time_duration);
    end
    a_avg = a_avg / trials;
    p = polyfit(time_duration(amount),a_avg(amount),1);
    f1 = polyval(p,time_duration);
    acc_start(j) = f1(1);
    plot(time_duration,f1);
    hold on;
%     plot(time_duration, smooth(a_avg, 9));
%     hold on;
%     plot(time_duration, a_avg);
end

figure;
plot(pwms, acc_start)
ylabel('Angular Acceleration')
xlabel('PWM value')
grid minor;

start_range1 = -1:0.02:-0.12;
start_range2 = 0.12:0.02:1;

hold on;
p1 = polyfit(start_range1,acc_start(1:1:length(start_range1)),1);
f1 = polyval(p1,pwms);
plot(pwms,f1);

p2 = polyfit(start_range2,acc_start(length(pwms)-length(start_range2)+1:1:length(pwms)),1);
f2 = polyval(p2,pwms);
plot(pwms,f2);

title('Angular Acceleration vs PWM');

%% No load stuff
writeDigitalPin(a, 'D6', 1);
writeDigitalPin(a, 'D7', 0);
writeDigitalPin(a, 'D9', 1);

no_load_voltage = 11.1;
no_load_current = 0.16;
no_load_power = no_load_voltage * no_load_current;

avg_speed = 0;
for i = 1:50
    speed = readSpeed(encoder);
    pause(0.1);
    avg_speed = avg_speed + speed;
end
avg_speed = avg_speed / 50;
avg_speed = (2 * pi) * avg_speed / 60;
writeDigitalPin(a, 'D9', 0);
torque = no_load_power / avg_speed;
moi_motor = torque / acc_start(1);

save ('data/torque_calculations.mat')

%% Torque stuff
clear acrobot;
close all;
acrobot = acrobot.acrobot_torque_control();
tau_range = -0.5:0.01:0.5;

% There should be a smooth speed transition
for i = 1:length(tau_range)
    pwm(i) = acrobot.getPWM(tau_range(i));
    if (pwm(i) > 0)
        writeDigitalPin(a, 'D6', 1);
        writeDigitalPin(a, 'D7', 0);
    else
        writeDigitalPin(a, 'D6', 0);
        writeDigitalPin(a, 'D7', 1);
    end

    writePWMDutyCycle(a,'D9',abs(pwm(i)));
    pause(0.1);
end
writePWMDutyCycle(a,'D9',0);
plot(tau_range, pwm);
grid minor;
xlabel('Tau Desired');
ylabel('PWM');

%% Arduino Rotary Encoder input
clear; clc;
a = arduino('/dev/ttyUSB0','Nano3','Libraries',{'RotaryEncoder', 'I2C'});
encoder = rotaryEncoder(a,'D2','D3');
rate = rateControl(10);
acrobot = acrobot.acrobot_torque_control();

%% Count Encoder
encoder.resetCount;
try
    while(1)
        position = readCount(encoder) / steps_per_rotation;
        pwm = acrobot.getPWM(position / 7);

        if (pwm > 0)
            writeDigitalPin(a, 'D6', 1);
            writeDigitalPin(a, 'D7', 0);
        else
            writeDigitalPin(a, 'D6', 0);
            writeDigitalPin(a, 'D7', 1);
        end

        writePWMDutyCycle(a,'D9',abs(pwm));

%        fprintf('Acc: %f %f %f Angvel: %f %f %f Power: %f\n',acc(1), acc(2), acc(3), angvel(1), angvel(2), angvel(3), power)

        waitfor(rate);
    end

catch ex
    disp(ex)
    writePWMDutyCycle(a,'D9',0);
end

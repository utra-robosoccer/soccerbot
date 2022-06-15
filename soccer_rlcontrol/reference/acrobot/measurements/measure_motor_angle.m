clear; clc;
a = arduino('/dev/ttyACM0','MKR1000','Libraries',{'RotaryEncoder', 'I2C'});
encoder = rotaryEncoder(a,'D0','D1');

while(1)
    count = readCount(encoder);
%     fprintf('Count: %d\n', count);
    measuredAngle = mod(((360 / 2797) * count), 360);
    fprintf('%d\n', (360 / 2797) * count);
%     fprintf('Current knob angle: %d\n', measuredAngle);
    pause(0.001);
end

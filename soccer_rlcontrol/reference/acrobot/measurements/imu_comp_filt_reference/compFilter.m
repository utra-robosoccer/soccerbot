function [] = compFilter(ard, dev, gyroCal, scaleFactorGyro, scaleFactorAcc, tau)

  % Set up and configure MPU
  writeRegister(dev, hex2dec('6B'), hex2dec('00'), 'int8'); % Activate mpu
  writeRegister(dev, hex2dec('1C'), hex2dec('08'), 'int8'); % Accelerometer
  writeRegister(dev, hex2dec('1B'), hex2dec('08'), 'int8'); % Gyroscope

  % Initialize values
  roll = 0;
  pitch = 0;
  yaw = 0;
  previous = 0;

  % Start timer and loop until stopped by user (or 30 secounds)
  tic

  % Run for 30 seconds
  while 1
    % Read from 6050
    [a g] = readMPU(dev, scaleFactorAcc, scaleFactorGyro, gyroCal);

    % Find angles from accelerometer
    accelPitch = rad2deg(atan2(a.y, a.z));
    accelRoll = rad2deg(atan2(a.x, a.z));

    % Calculate dt
    dt = toc - previous;
    previous = toc;

    % Apply complementary filter
    pitch = (tau)*(pitch + g.x * dt) + (1 - tau)*(accelPitch);
    roll = (tau)*(roll - g.y * dt) + (1 - tau)*(accelRoll);
    yaw = (yaw + g.z * dt);

    % Print results
    fprintf('Roll: %0.3f, Pitch: %0.3f, Yaw: %0.3f\n',roll, pitch, yaw)
  end

end

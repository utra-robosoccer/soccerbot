function [gyroCal] = calibrateGyro(dev, scaleFactorGyro)

  % Set up and configure MPU
  writeRegister(dev, hex2dec('6B'), hex2dec('00'), 'int8'); % Activate MPU
  writeRegister(dev, hex2dec('1C'), hex2dec('08'), 'int8'); % Accelerometer
  writeRegister(dev, hex2dec('1B'), hex2dec('08'), 'int8'); % Gyroscope

  % Talk to user and pause
  fprintf('Hold IMU still, calibrating...')
  pause(2)

  % Start counter and timer
  i = 0;
  tic
  timer = toc;

  % Loop through gyro values for 15 seconds
  while toc < 15
    i = i + 1;

    g = readGyro(dev, scaleFactorGyro);
    gyroCalX(i) = g.x;
    gyroCalY(i) = g.y;
    gyroCalZ(i) = g.z;

    if (toc - timer) > 0.5
      fprintf('.')
      timer = toc;
    end
  end

  fprintf('\nCalibration complete\n')

  % Calculate average offset
  gyroCal.x = sum(gyroCalX) / length(gyroCalX);
  gyroCal.y = sum(gyroCalY) / length(gyroCalY);
  gyroCal.z = sum(gyroCalZ) / length(gyroCalZ);

  % Display info to user
  fprintf('%0.0f values sampled\n',length(gyroCalX))

end


function [g] = readGyro(dev, scaleFactorGyro)
  % Get raw int8 values from the registry(s)
  GYRO_XOUT_H = readRegister(dev,hex2dec('43'),'int8');
  GYRO_XOUT_L = readRegister(dev,hex2dec('44'),'int8');

  GYRO_YOUT_H = readRegister(dev,hex2dec('45'),'int8');
  GYRO_YOUT_L = readRegister(dev,hex2dec('46'),'int8');

  GYRO_ZOUT_H = readRegister(dev,hex2dec('47'),'int8');
  GYRO_ZOUT_L = readRegister(dev,hex2dec('48'),'int8');

  % Cast the low and high int8 values to usable int16 values
  g.x = double(typecast(int8([GYRO_XOUT_L GYRO_XOUT_H]),'int16')) / scaleFactorGyro;
  g.y = double(typecast(int8([GYRO_YOUT_L GYRO_YOUT_H]),'int16')) / scaleFactorGyro;
  g.z = double(typecast(int8([GYRO_ZOUT_L GYRO_ZOUT_H]),'int16')) / scaleFactorGyro;
end

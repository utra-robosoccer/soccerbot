if (exist('ard','var') ~= 1)
  % Connect to arduino
  ard = arduino;
  dev = i2cdev(ard, '0x68');

  % Set up and configure MPU for +/- 4g and 500 deg/s
  scaleFactorGyro = 65.5;
  scaleFactorAcc = 819.2;

  % Calibrate the gyroscope
  gyroCal = calibrateGyro(dev, scaleFactorGyro);

  % Run the comp filter
  compFilter(ard, dev, gyroCal, scaleFactorGyro, scaleFactorAcc, 0.98)
else
  % If already started once and variables are saved just run the filter
  compFilter(ard, dev, gyroCal, scaleFactorGyro, scaleFactorAcc, 0.98)
end

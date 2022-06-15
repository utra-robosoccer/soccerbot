function [a g] = readMPU(dev, scaleFactorAccel, scaleFactorGyro, gyroCal)

  % Read Accelerometer
  ACCEL_XOUT_H = readRegister(dev,hex2dec('3B'),'int8');
  ACCEL_XOUT_L = readRegister(dev,hex2dec('3C'),'int8');

  ACCEL_YOUT_H = readRegister(dev,hex2dec('3D'),'int8');
  ACCEL_YOUT_L = readRegister(dev,hex2dec('3E'),'int8');

  ACCEL_ZOUT_H = readRegister(dev,hex2dec('3F'),'int8');
  ACCEL_ZOUT_L = readRegister(dev,hex2dec('40'),'int8');

  a.x = double(typecast(int8([ACCEL_XOUT_L ACCEL_XOUT_H]),'int16')) / scaleFactorAccel;
  a.y = double(typecast(int8([ACCEL_YOUT_L ACCEL_YOUT_H]),'int16')) / scaleFactorAccel;
  a.z = double(typecast(int8([ACCEL_ZOUT_L ACCEL_ZOUT_H]),'int16')) / scaleFactorAccel;

  % Read Gyroscope
  GYRO_XOUT_H = readRegister(dev,hex2dec('43'),'int8');
  GYRO_XOUT_L = readRegister(dev,hex2dec('44'),'int8');

  GYRO_YOUT_H = readRegister(dev,hex2dec('45'),'int8');
  GYRO_YOUT_L = readRegister(dev,hex2dec('46'),'int8');

  GYRO_ZOUT_H = readRegister(dev,hex2dec('47'),'int8');
  GYRO_ZOUT_L = readRegister(dev,hex2dec('48'),'int8');

  g.x = (double(typecast(int8([GYRO_XOUT_L GYRO_XOUT_H]),'int16')) / scaleFactorGyro) - gyroCal.x;
  g.y = (double(typecast(int8([GYRO_YOUT_L GYRO_YOUT_H]),'int16')) / scaleFactorGyro) - gyroCal.y;
  g.z = (double(typecast(int8([GYRO_ZOUT_L GYRO_ZOUT_H]),'int16')) / scaleFactorGyro) - gyroCal.z;

end

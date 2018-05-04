#include "MPU6050.hpp"



MPU6050_i2cDev::MPU6050_i2cDev(uint8_t address)
  : _mpu6050(address), _sensitivity(1) {}

bool MPU6050_i2cDev::initialize() {
  // Wire.begin();
  // Wire.setClock(800000); // max: 888888
  // Fastwire::setup(800, true);
  // delay(10);


  _mpu6050.initialize();
  for (auto i = 0; i < 2; ++i) {

// DLPF
//          |   ACCELEROMETER    |           GYROSCOPE
// DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
// ---------+-----------+--------+-----------+--------+-------------
// 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
// 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
// 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
// 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
// 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
// 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
// 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
// 7        |   -- Reserved --   |   -- Reserved --   | Reserved

// Full scale gyro range
// FS_SEL | Full Scale Range   | LSB Sensitivity
// -------+--------------------+----------------
// 0      | +/- 250 degrees/s  | 131 LSB/deg/s
// 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
// 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
// 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s

// Full scale accel range
// AFS_SEL | Full Scale Range | LSB Sensitivity
// --------+------------------+----------------
// 0       | +/- 2g           | 16384 LSB/mg
// 1       | +/- 4g           | 8192 LSB/mg
// 2       | +/- 8g           | 4096 LSB/mg
// 3       | +/- 16g          | 2048 LSB/mg

    setDLPFMode(1);
    setSensitivity(1);

    // Protoboard
    // _mpu6050.setXAccelOffset(-5493);
    // _mpu6050.setYAccelOffset(-6087);
    // _mpu6050.setZAccelOffset(1665);
    // _mpu6050.setXGyroOffset(81);
    // _mpu6050.setYGyroOffset(-4);
    // _mpu6050.setZGyroOffset(-40);

    // MAD-FC-2 board with legs
    // _mpu6050.setXAccelOffset(-5317);
    // _mpu6050.setYAccelOffset(-6029);
    // _mpu6050.setZAccelOffset(1567);
    // _mpu6050.setXGyroOffset(78);
    // _mpu6050.setYGyroOffset(-9);
    // _mpu6050.setZGyroOffset(-15);

    // MAD-FC-2 board
    // _mpu6050.setXAccelOffset(-5255);
    // _mpu6050.setYAccelOffset(-5932);
    // _mpu6050.setZAccelOffset(1570);
    // _mpu6050.setXGyroOffset(77);
    // _mpu6050.setYGyroOffset(-5);
    // _mpu6050.setZGyroOffset(-14);

    // MAD-FC-2 board
    // _mpu6050.setXAccelOffset(-5252);
    // _mpu6050.setYAccelOffset(-6024);
    // _mpu6050.setZAccelOffset(1576);
    // _mpu6050.setXGyroOffset(76);
    // _mpu6050.setYGyroOffset(-17);
    // _mpu6050.setZGyroOffset(-13);

    // MAD-FC-2 board with one broken motor and broken central top frame
    // _mpu6050.setXAccelOffset(-5345);
    // _mpu6050.setYAccelOffset(-6083);
    // _mpu6050.setZAccelOffset(1576);
    // _mpu6050.setXGyroOffset(77);
    // _mpu6050.setYGyroOffset(-16);
    // _mpu6050.setZGyroOffset(-12);

    // MAD-FC-2 board with one broken motor
    // _mpu6050.setXAccelOffset(-5243);
    // _mpu6050.setYAccelOffset(-5938);
    // _mpu6050.setZAccelOffset(1605);
    // _mpu6050.setXGyroOffset(77);
    // _mpu6050.setYGyroOffset(-17);
    // _mpu6050.setZGyroOffset(-13);



    // MAD-FC-2 board with one broken motor and glued IMU

    // T=8.11
    // _mpu6050.setXAccelOffset(-5256);
    // _mpu6050.setYAccelOffset(-6012);
    // _mpu6050.setZAccelOffset(1474);
    // _mpu6050.setXGyroOffset(69);
    // _mpu6050.setYGyroOffset(-15);
    // _mpu6050.setZGyroOffset(-9);
    // T=8.18
    // _mpu6050.setXAccelOffset(-5258);
    // _mpu6050.setYAccelOffset(-6012);
    // _mpu6050.setZAccelOffset(1475);
    // _mpu6050.setXGyroOffset(69);
    // _mpu6050.setYGyroOffset(-15);
    // _mpu6050.setZGyroOffset(-9);
    // T=8.27
    // _mpu6050.setXAccelOffset(-5258);
    // _mpu6050.setYAccelOffset(-6012);
    // _mpu6050.setZAccelOffset(1476);
    // _mpu6050.setXGyroOffset(70);
    // _mpu6050.setYGyroOffset(-15);
    // _mpu6050.setZGyroOffset(-9);
    // T=8.36
    // _mpu6050.setXAccelOffset(-5258);
    // _mpu6050.setYAccelOffset(-6012);
    // _mpu6050.setZAccelOffset(1476);
    // _mpu6050.setXGyroOffset(70);
    // _mpu6050.setYGyroOffset(-15);
    // _mpu6050.setZGyroOffset(-9);
    // T=8.48
    // _mpu6050.setXAccelOffset(-5258);
    // _mpu6050.setYAccelOffset(-6012);
    // _mpu6050.setZAccelOffset(1478);
    // _mpu6050.setXGyroOffset(70);
    // _mpu6050.setYGyroOffset(-15);
    // _mpu6050.setZGyroOffset(-9);
    // T=9.05
    // _mpu6050.setXGyroOffset(70);
    // _mpu6050.setYGyroOffset(-15);
    // _mpu6050.setZGyroOffset(-9);
    // T=9.44
    // _mpu6050.setXGyroOffset(71);
    // _mpu6050.setYGyroOffset(-15);
    // _mpu6050.setZGyroOffset(-9);
    // T=10.02
    // _mpu6050.setXGyroOffset(71);
    // _mpu6050.setYGyroOffset(-15);
    // _mpu6050.setZGyroOffset(-9);
    // T=10.55
    // _mpu6050.setXGyroOffset(72);
    // _mpu6050.setYGyroOffset(-15);
    // _mpu6050.setZGyroOffset(-9);
    // T=10.79
    //_mpu6050.setXGyroOffset(72);
    //_mpu6050.setYGyroOffset(-15);
    //_mpu6050.setZGyroOffset(-9);
    // T=12.21
    //_mpu6050.setXGyroOffset(74);
    //_mpu6050.setYGyroOffset(-15);
    //_mpu6050.setZGyroOffset(-9);
    // T=12.53
    //_mpu6050.setXGyroOffset(74);
    //_mpu6050.setYGyroOffset(-15);
    //_mpu6050.setZGyroOffset(-9);
    // T=12.69
    //_mpu6050.setXGyroOffset(74);
    //_mpu6050.setYGyroOffset(-15);
    //_mpu6050.setZGyroOffset(-9);
    // T=12.71
    //_mpu6050.setXGyroOffset(74);
    //_mpu6050.setYGyroOffset(-15);
    //_mpu6050.setZGyroOffset(-9);
//  [-5457,-5456] --> [-14,4]  [-6053,-6052] --> [-5,14] [1493,1494] --> [16363,16387] [70,71] --> [-2,2]  [-26,-25] --> [-3,1]  [-1,0] --> [-5,7]
// 14.32
// [-5457,-5456] --> [-3,14] [-6053,-6052] --> [-19,1] [1501,1502] --> [16375,16395] [71,72] --> [-1,2]  [-25,-25] --> [0,1] [-1,0] --> [-6,8]
// 15.50
// [-5423,-5422] --> [-14,4] [-6037,-6036] --> [-4,14] [1521,1522] --> [16380,16398] [75,76] --> [-1,2]  [-25,-24] --> [-1,3]  [4,5] --> [0,3]
// 15.11
// [-5429,-5428] --> [-2,16] [-6109,-6108] --> [-12,5] [1511,1512] --> [16379,16392] [77,78] --> [-1,2]  [-24,-23] --> [0,3] [24,25] --> [0,2]
// 14.50
// [-5401,-5400] --> [-2,16] [-6073,-6072] --> [-9,6]  [1525,1526] --> [16372,16388] [77,78] --> [-1,2]  [-25,-24] --> [0,3] [11,12] --> [-2,1]
// 16.70

    // _mpu6050.setXAccelOffset(-5428);
    // _mpu6050.setYAccelOffset(-6040);
    // _mpu6050.setZAccelOffset(1490); 8192
    // _mpu6050.setXGyroOffset(68);
    // _mpu6050.setYGyroOffset(-25);
    // _mpu6050.setZGyroOffset(-0);
    _mpu6050.setXAccelOffset(-5400); //19
    _mpu6050.setYAccelOffset(-6073); //130
    _mpu6050.setZAccelOffset(1526);
    _mpu6050.setXGyroOffset(77);
    _mpu6050.setYGyroOffset(-24);
    _mpu6050.setZGyroOffset(11);
  }
  return _mpu6050.testConnection();
}

void MPU6050_i2cDev::setDLPFMode(int8_t mode) {
  if (mode < 0) mode = 0;
  else if (mode > 6) mode = 6;
  _mpu6050.setDLPFMode(mode);
}

void MPU6050_i2cDev::setSensitivity(int8_t sensitivity) {
  if (sensitivity < 0) sensitivity = 0;
  else if (sensitivity > 3) sensitivity = 3;
  _mpu6050.setFullScaleGyroRange(sensitivity);
  _mpu6050.setFullScaleAccelRange(sensitivity);
  _sensitivity = sensitivity;
}

float MPU6050_i2cDev::toGForce(int16_t acceleration) {
  return acceleration / MPU6050_i2cDev::_accelSensitivity[_sensitivity];
}

void MPU6050_i2cDev::getAngularVelocity(AngularVelocity* angularVelocity) {
  int16_t xg, yg, zg;
  _mpu6050.getRotation(&xg, &yg, &zg);
  angularVelocity->y = -xg / 65.5;
  angularVelocity->x = 0 - (yg / 65.5);
  angularVelocity->z = zg / 65.5;
}

void MPU6050_i2cDev::getAcceleration(Acceleration* acceleration) {
  _mpu6050.getAcceleration(&acceleration->y, &acceleration->x, &acceleration->z);
}

void MPU6050_i2cDev::getMotion(AngularVelocity* angularVelocity, Acceleration* acceleration) {
  int16_t xg, yg, zg;
  Acceleration accelTemp;
  _mpu6050.getMotion6(&accelTemp.x, &accelTemp.y, &accelTemp.z, &xg, &yg, &zg);
  if (accelTemp.y == 0 && accelTemp.z == 0 && xg == 0 && yg == 0) return;
  acceleration->x = accelTemp.x;
  acceleration->y = accelTemp.y;
  acceleration->z = accelTemp.z;
  angularVelocity->x = xg / 65.5;
  angularVelocity->y = yg / 65.5;
  angularVelocity->z = zg / 65.5;
}

void MPU6050_i2cDev::getRawMotion(int16_t* xa, int16_t* ya, int16_t* za, int16_t* xg, int16_t* yg, int16_t* zg) {
  _mpu6050.getMotion6(xa, ya, za, xg, yg, zg);
}

float MPU6050_i2cDev::getTemperature() {
  return (float)_mpu6050.getTemperature() / 340.0 + 36.53;
}

void MPU6050_i2cDev::updateOffsets() {
  float temperature = getTemperature();
  if (temperature > 0 && temperature < 15) {

  }
  _mpu6050.setXAccelOffset(-5258);
  _mpu6050.setYAccelOffset(-6012);
  _mpu6050.setZAccelOffset(1478);
  _mpu6050.setXGyroOffset(73);
  _mpu6050.setYGyroOffset(-15);
  _mpu6050.setZGyroOffset(-9);
}

float MPU6050_i2cDev::_gyroSensitivity[] = {131.0, 65.5, 32.8, 16.4};
int16_t MPU6050_i2cDev::_accelSensitivity[] = {16384, 8192, 4096, 2048};
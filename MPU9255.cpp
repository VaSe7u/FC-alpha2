#include "MPU9255.hpp"

MPU9255::MPU9255(uint8_t address) : _mpu9255(address) {}

bool MPU9255::initialize() {
  delay(10);
  bool s = true;
  _mpu9255.initialize();
  for (auto i = 0; i < 3; ++i) {
    s &= _mpu9255.setAccelDLPF(2);
    s &= _mpu9255.setAccelFullScaleRange(1);
    s &= _mpu9255.setGyroDLPF(2);
    s &= _mpu9255.setGyroFullScaleRange(1);

    // _mpu9255.setXAccelOffset(2587);
    // _mpu9255.setYAccelOffset(-2474);
    // _mpu9255.setZAccelOffset(5226);
    // _mpu9255.setXGyroOffset (6);
    // _mpu9255.setYGyroOffset (-7);
    // _mpu9255.setZGyroOffset (2);


    _mpu9255.setXAccelOffset(2584);
    _mpu9255.setYAccelOffset(-2490);
    _mpu9255.setZAccelOffset(5221);
    _mpu9255.setXGyroOffset (5);
    _mpu9255.setYGyroOffset (-6);
    _mpu9255.setZGyroOffset (8);
  }
  s &= _mpu9255.testConnection();
  return s;
}

void MPU9255::getMotion(AngularVelocity* angularVelocity, Acceleration* acceleration) {
  int16_t xg = 0, yg = 0, zg = 0;
  Acceleration accelTemp;
  accelTemp.x = 0;
  accelTemp.y = 0;
  accelTemp.z = 0;
  _mpu9255.getAccelAndGyroRaw(&accelTemp.x, &accelTemp.y, &accelTemp.z, &xg, &yg, &zg);
  if (accelTemp.y == 0 && accelTemp.z == 0 && xg == 0 && yg == 0) return;
  acceleration->x = accelTemp.x;
  acceleration->y = accelTemp.y;
  acceleration->z = accelTemp.z;
  angularVelocity->x = xg / 65.5;
  angularVelocity->y = yg / 65.5;
  angularVelocity->z = zg / 65.5;
}

float MPU9255::getTemperature() {
  float temperature = 0.0f;
  _mpu9255.getTemperature(&temperature);
  return temperature;
}
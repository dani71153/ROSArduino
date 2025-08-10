// Mpu9250Tca.h
#pragma once
#include <Wire.h>
#include <MPU9250_asukiaaa.h>

class Mpu9250Tca {
public:
  Mpu9250Tca(TwoWire& bus, uint8_t tcaAddr, uint8_t mpuAddr, uint8_t tcaChannel, uint32_t i2cHz=400000)
  : wire(bus), TCA_ADDR(tcaAddr), MPU_ADDR(mpuAddr), TCA_CHANNEL(tcaChannel),
    imu(mpuAddr), i2cClock(i2cHz) {}

  void begin(unsigned long serialBaud=0) {
    if (serialBaud) Serial.begin(serialBaud);
    wire.begin();
    wire.setClock(i2cClock);
    tcaSelect();
    imu.setWire(&wire);
    imu.beginAccel();
    imu.beginGyro();
    imu.beginMag();
  }

  void resetI2C() {
    // Algunos cores no implementan end(); si no existe, elimina esta línea.
    wire.end();
    delay(2);
    wire.begin();
    wire.setClock(i2cClock);
    tcaSelect();
    imu.beginAccel();
    imu.beginGyro();
    imu.beginMag();
  }

  void update() {
    tcaSelect();                         // asegura canal correcto
    imu.accelUpdate();  ax = imu.accelX(); ay = imu.accelY(); az = imu.accelZ();
    imu.gyroUpdate();   gx = imu.gyroX();  gy = imu.gyroY();  gz = imu.gyroZ();
    imu.magUpdate();    mx = imu.magX();   my = imu.magY();   mz = imu.magZ();
    tempC = readTemperatureC();          // usa mismo canal
  }

  void printSerial() {                   // solo imprime cache
    Serial.print("<");
    Serial.print("A:"); Serial.print(ax); Serial.print(","); Serial.print(ay); Serial.print(","); Serial.print(az); Serial.print(";");
    Serial.print("G:"); Serial.print(gx); Serial.print(","); Serial.print(gy); Serial.print(","); Serial.print(gz); Serial.print(";");
    Serial.print("M:"); Serial.print(mx); Serial.print(","); Serial.print(my); Serial.print(","); Serial.print(mz); Serial.print(";");
    Serial.print("T:"); Serial.print(tempC, 2);
    Serial.println(">");
  }

private:
  TwoWire& wire;
  const uint8_t TCA_ADDR, MPU_ADDR, TCA_CHANNEL;
  MPU9250_asukiaaa imu;
  uint32_t i2cClock;

  // cache
  float ax=0, ay=0, az=0, gx=0, gy=0, gz=0, mx=0, my=0, mz=0, tempC=NAN;

  void tcaSelect() {
    wire.beginTransmission(TCA_ADDR);
    wire.write(1 << TCA_CHANNEL);
    wire.endTransmission();
    delay(1); // settle suficiente con TCA9548A
  }

  float readTemperatureC() {
    wire.beginTransmission(MPU_ADDR);
    wire.write(0x41);                   // TEMP_OUT_H
    wire.endTransmission(false);
    wire.requestFrom((int)MPU_ADDR, 2);
    if (wire.available() < 2) return NAN;
    uint8_t h = wire.read(), l = wire.read();
    int16_t raw = (int16_t)((h << 8) | l);
    return ((float)raw / 333.87f) + 21.0f;
  }
};

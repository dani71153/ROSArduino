#include "mpu9250only.h"

// ===== TCA helpers (canal 0 fijo) =====
void tcaselect(uint8_t ch){
  if (ch > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1u << ch);
  Wire.endTransmission();      // deja seleccionado ch
}
void tcadeselectAll(){
  Wire.beginTransmission(TCAADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}

// ===== Estado =====
static MPU9250_asukiaaa mpu(0x68);
static bool g_inited = false;

// rd16 rápido (para temperatura)
static inline bool rd16(uint8_t addr, uint8_t reg, int16_t& v){
  Wire.beginTransmission(addr); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 2) != 2) return false;
  uint8_t msb = Wire.read(), lsb = Wire.read();
  v = (int16_t)((msb << 8) | lsb);
  return true;
}

void sensores_init(){
  Wire.begin();
  Wire.setClock(400000);     // 400 kHz
  delay(10);

  // Selecciona TCA canal 0 y déjalo así
  tcaselect(2);

  // Wake + bypass AK8963 (necesario para mag)
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x01); Wire.endTransmission(); // wake
  Wire.beginTransmission(0x68); Wire.write(0x6A); Wire.write(0x00); Wire.endTransmission(); // I2C master OFF
  Wire.beginTransmission(0x68); Wire.write(0x37); Wire.write(0x02); Wire.endTransmission(); // BYPASS=1

  mpu.setWire(&Wire);
  mpu.beginAccel();
  mpu.beginGyro();
  mpu.beginMag();

  g_inited = true;
}

bool read_mpu(MpuReading* out, bool emitSerial){
  if (!g_inited) return false;

  // Asegura TCA en 0 por si algo lo cambió
  tcaselect(2);

  if (mpu.accelUpdate()!=0 || mpu.gyroUpdate()!=0 || mpu.magUpdate()!=0){
    if (emitSerial) Serial.println(F("MPU,ERR"));
    if (out) *out = MpuReading{};
    return false;
  }

  MpuReading r;
  r.valid = true;
  r.ax = mpu.accelX(); r.ay = mpu.accelY(); r.az = mpu.accelZ();
  r.gx = mpu.gyroX();  r.gy = mpu.gyroY();  r.gz = mpu.gyroZ();
  r.mx = mpu.magX();   r.my = mpu.magY();   r.mz = mpu.magZ();

  int16_t tRaw;
  if (rd16(0x68, 0x41, tRaw)) r.tempC = (float)tRaw / 333.87f + 21.0f;

  if (emitSerial){
    Serial.print(F("A:")); Serial.print(r.ax); Serial.write(',');
    Serial.print(r.ay); Serial.write(','); Serial.print(r.az);
    Serial.print(F(";G:")); Serial.print(r.gx); Serial.write(',');
    Serial.print(r.gy); Serial.write(','); Serial.print(r.gz);
    Serial.print(F(";M:")); Serial.print(r.mx); Serial.write(',');
    Serial.print(r.my); Serial.write(','); Serial.print(r.mz);
    Serial.print(F(";T:")); if (isnan(r.tempC)) Serial.print(F("nan")); else Serial.print(r.tempC,2);
    Serial.println();
  }

  if (out) *out = r;
  return true;
}

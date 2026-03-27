#include <mpu9250&bno055.h>

#define DEBUG_TCA 1
#define DEBUG_I2C 1

void tcaselect(uint8_t i){
#if DEBUG_TCA
  Serial.print("[TCA] Seleccionando canal "); Serial.println(i);
#endif
  if(i>7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void tcadeselectAll(){
#if DEBUG_TCA
  Serial.println("[TCA] Deseleccionando todos los canales");
#endif
  Wire.beginTransmission(TCAADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}

bool rd8(uint8_t addr,uint8_t reg,uint8_t& val){
#if DEBUG_I2C
  Serial.print("[I2C] Leyendo 0x"); Serial.print(addr, HEX); Serial.print(" reg 0x"); Serial.println(reg, HEX);
#endif
  Wire.beginTransmission(addr); 
  Wire.write(reg);
  if(Wire.endTransmission(false)!=0) {
#if DEBUG_I2C
    Serial.println("[I2C] endTransmission fail");
#endif
    return false;
  }
  if(Wire.requestFrom((int)addr,1)!=1) {
#if DEBUG_I2C
    Serial.println("[I2C] requestFrom fail");
#endif
    return false;
  }
  val = Wire.read(); 
#if DEBUG_I2C
  Serial.print("[I2C] Valor leído: "); Serial.println(val, HEX);
#endif
  return true;
}

bool wr8(uint8_t addr,uint8_t reg,uint8_t val){
#if DEBUG_I2C
  Serial.print("[I2C] Escribiendo 0x"); Serial.print(addr, HEX); Serial.print(" reg 0x"); Serial.print(reg, HEX); Serial.print(" val 0x"); Serial.println(val, HEX);
#endif
  Wire.beginTransmission(addr); 
  Wire.write(reg); 
  Wire.write(val);
  bool ok = (Wire.endTransmission(true)==0);
#if DEBUG_I2C
  Serial.print("[I2C] Write ok: "); Serial.println(ok ? "YES":"NO");
#endif
  return ok;
}

void mpu_quarantine(uint8_t a){
#if DEBUG_I2C
  Serial.println("[MPU] Quarantine");
#endif
  wr8(a,0x6A,0x00);
  wr8(a,0x37,0x00);
  wr8(a,0x6B,0x40);
}

void read_mpu_on_channel(uint8_t ch) {
  uint8_t who=0, mpuAddr=0;
  tcaselect(ch);

  Serial.print("[MPU] Intentando ID en 0x68: ");
  if(rd8(0x68,0x75,who)) Serial.println(who, HEX);
  if(rd8(0x68,0x75,who) && (who==0x71||who==0x73)) mpuAddr=0x68;
  else {
    Serial.print("[MPU] Intentando ID en 0x69: ");
    if(rd8(0x69,0x75,who)) Serial.println(who, HEX);
    if(rd8(0x69,0x75,who) && (who==0x71||who==0x73)) mpuAddr=0x69;
  }

  if (!mpuAddr) {
    Serial.print("MPU,"); Serial.print(ch); Serial.println(",NA");
    tcadeselectAll();
    return;
  }

  wr8(mpuAddr,0x6B,0x80);
  unsigned long t0 = millis();
  while(millis()-t0 < 6) {}

  wr8(mpuAddr,0x6B,0x01);

  MPU9250_asukiaaa mpu(mpuAddr);
  mpu.setWire(&Wire);
  mpu.beginAccel(); mpu.beginGyro(); mpu.beginMag();
  mpu.accelUpdate(); mpu.gyroUpdate(); mpu.magUpdate();

  Serial.print("MPU,"); Serial.print(ch); Serial.print(",A:");
  Serial.print(mpu.accelX()); Serial.print(','); Serial.print(mpu.accelY()); Serial.print(','); Serial.print(mpu.accelZ());
  Serial.print(",G:"); Serial.print(mpu.gyroX()); Serial.print(','); Serial.print(mpu.gyroY()); Serial.print(','); Serial.print(mpu.gyroZ());
  Serial.print(",M:"); Serial.print(mpu.magX()); Serial.print(','); Serial.print(mpu.magY()); Serial.print(','); Serial.println(mpu.magZ());

  mpu_quarantine(mpuAddr);
  tcadeselectAll();
}

void read_bno_on_channel(uint8_t ch) {
  uint8_t id=0, addr=0;
  tcaselect(ch);
  Serial.print("[BNO] Probando 0x29... ");
  if(rd8(0x29,0x00,id)) {
    Serial.print("id: "); Serial.println(id, HEX);
    if(id == 0xA0) addr=0x29;
  }
  if(!addr) {
    Serial.print("[BNO] Probando 0x28... ");
    if(rd8(0x28,0x00,id)) {
      Serial.print("id: "); Serial.println(id, HEX);
      if(id == 0xA0) addr=0x28;
    }
  }
  if(!addr) {
    Serial.print("BNO,"); Serial.print(ch); Serial.println(",NA");
    tcadeselectAll();
    return;
  }

  Adafruit_BNO055 bno(55, addr, &Wire);
  if(!bno.begin()) { 
    Serial.print("BNO,"); Serial.print(ch); Serial.println(",NA"); 
    tcadeselectAll(); 
    return; 
  }

  bno.setMode(OPERATION_MODE_CONFIG);
  bno.setMode(OPERATION_MODE_NDOF);

  imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> g = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  Serial.print("BNO,"); Serial.print(ch); Serial.print(",E:");
  Serial.print(e.x()); Serial.print(','); Serial.print(e.y()); Serial.print(','); Serial.print(e.z());
  Serial.print(",A:");   Serial.print(a.x()); Serial.print(','); Serial.print(a.y()); Serial.print(','); Serial.print(a.z());
  Serial.print(",G:");   Serial.print(g.x()); Serial.print(','); Serial.print(g.y()); Serial.print(','); Serial.println(g.z());

  bno.setMode(OPERATION_MODE_CONFIG);
  tcadeselectAll();
}

// Función de inicialización opcional para llamar desde tu setup()
void sensores_init(){
  Wire.begin(); 
  Wire.setClock(I2C_KHZ);
  tcadeselectAll();
  Serial.println("Sensores listos.");

  // Escaneo básico de canales esperados y direcciones conocidas
  const uint8_t canales[] = {0, 2};
  const uint8_t dir_bno[] = {0x28, 0x29};
  const uint8_t dir_mpu[] = {0x68, 0x69};

  for (uint8_t i = 0; i < sizeof(canales)/sizeof(canales[0]); i++) {
    uint8_t ch = canales[i];
    tcaselect(ch);
    Serial.print(" Canal ");
    Serial.print(ch);

    // Buscar BNO055
    bool bno_ok = false;
    for (uint8_t j = 0; j < sizeof(dir_bno)/sizeof(dir_bno[0]); j++) {
      Wire.beginTransmission(dir_bno[j]);
      if (Wire.endTransmission() == 0) {
        Serial.print(" | BNO055 encontrado en 0x");
        Serial.print(dir_bno[j], HEX);
        bno_ok = true;
      }
    }
    if (!bno_ok) Serial.print(" | BNO055: NA");

    // Buscar MPU9250
    bool mpu_ok = false;
    for (uint8_t j = 0; j < sizeof(dir_mpu)/sizeof(dir_mpu[0]); j++) {
      Wire.beginTransmission(dir_mpu[j]);
      if (Wire.endTransmission() == 0) {
        Serial.print(" | MPU9250 encontrado en 0x");
        Serial.print(dir_mpu[j], HEX);
        mpu_ok = true;
      }
    }
    if (!mpu_ok) Serial.print(" | MPU9250: NA");

    Serial.println();
    tcadeselectAll();
  }
}

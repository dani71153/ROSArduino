#include <mpu9250&bno055.h>

#ifndef I2C_KHZ
#define I2C_KHZ 400000UL
#endif

// ---- TCA ----
static inline void tcaselect(uint8_t i){
  if(i>7) return;
  Wire.beginTransmission(TCAADDR); Wire.write(1<<i); Wire.endTransmission();
  delayMicroseconds(500);
}
static inline void tcadeselectAll(){
  Wire.beginTransmission(TCAADDR); Wire.write(0x00); Wire.endTransmission();
}

// ---- I2C helpers ----
static inline bool rd8(uint8_t a,uint8_t r,uint8_t& v){
  Wire.beginTransmission(a); Wire.write(r);
  if(Wire.endTransmission(false)!=0) return false;
  if(Wire.requestFrom((int)a,1)!=1)   return false;
  v=Wire.read(); return true;
}
static inline bool rd16(uint8_t a,uint8_t r,int16_t& v){
  uint8_t msb, lsb;
  if(!rd8(a,r,msb)) return false;
  if(!rd8(a,(uint8_t)(r+1),lsb)) return false;
  v = (int16_t)((msb<<8) | lsb);
  return true;
}
static inline bool wr8(uint8_t a,uint8_t r,uint8_t v){
  Wire.beginTransmission(a); Wire.write(r); Wire.write(v);
  return Wire.endTransmission(true)==0;
}

// ---- MPU quarantine ----
static inline void mpu_quarantine(uint8_t a){
  wr8(a,0x6A,0x00); // USER_CTRL: I2C master off
  wr8(a,0x37,0x00); // INT_PIN_CFG: bypass off
  wr8(a,0x6B,0x40); // PWR_MGMT_1: sleep on
  delayMicroseconds(100);
}

// Despierta y habilita bypass AK8963 (para mag) en cada ciclo
static inline void mpu_wake_and_bypass(uint8_t a){
  wr8(a, 0x6B, 0x01); // PWR_MGMT_1: wake, clock auto
  wr8(a, 0x6A, 0x00); // USER_CTRL: I2C master OFF (necesario para bypass)
  wr8(a, 0x37, 0x02); // INT_PIN_CFG: I2C_BYPASS_EN=1
  delayMicroseconds(200);
}

// ---- Caches mínimas ----
static uint8_t  g_mpuAddr = 0;
static MPU9250_asukiaaa* g_mpu = nullptr;
static bool     g_mpu_inited = false;

static uint8_t  g_bnoAddr = 0;
static Adafruit_BNO055* g_bno = nullptr;
static bool     g_bno_inited = false;

// ---- Detectores ----
static bool detect_mpu(uint8_t &addr){
  uint8_t who=0;
  if(rd8(0x68,0x75,who) && (who==0x71||who==0x73)){ addr=0x68; return true; }
  if(rd8(0x69,0x75,who) && (who==0x71||who==0x73)){ addr=0x69; return true; }
  return false;
}
static bool detect_bno(uint8_t &addr){
  uint8_t id=0;
  if(rd8(0x29,0x00,id) && id==0xA0){ addr=0x29; return true; }
  if(rd8(0x28,0x00,id) && id==0xA0){ addr=0x28; return true; }
  return false;
}

// ---- Temp MPU9250 en °C (reg 0x41-0x42)
static bool mpu_read_tempC(uint8_t addr, float &tempC){
  int16_t raw;
  if(!rd16(addr, 0x41, raw)) return false;
  tempC = (float)raw / 333.87f + 21.0f; // especificación MPU9250
  return true;
}

// ======================= MPU9250 (canal 2) =======================


void read_mpu_on_channel(uint8_t ch) {
  if (ch!=2){ Serial.print("MPU,"); Serial.print(ch); Serial.println(",NA"); return; }

  tcaselect(2);

  if (!g_mpuAddr && !detect_mpu(g_mpuAddr)){
    Serial.println("MPU,2,NA"); tcadeselectAll(); return;
  }
  if (!g_mpu){
    g_mpu = new MPU9250_asukiaaa(g_mpuAddr);
    g_mpu->setWire(&Wire);
  }
  if (!g_mpu_inited){
    // init una sola vez (sin quarantine aquí)
    wr8(g_mpuAddr,0x6B,0x01);                  // wake
    g_mpu->beginAccel(); g_mpu->beginGyro(); g_mpu->beginMag();
    g_mpu_inited = true;
  }

  // 🔧 clave: salir de sleep y habilitar bypass cada ciclo
  mpu_wake_and_bypass(g_mpuAddr);

  g_mpu->accelUpdate();
  g_mpu->gyroUpdate();
  g_mpu->magUpdate();

  float tempC = NAN;
  mpu_read_tempC(g_mpuAddr, tempC);

  // ==== FORMATO ORIGINAL (ROS) ====
  Serial.print("A:"); Serial.print(g_mpu->accelX()); Serial.print(","); Serial.print(g_mpu->accelY()); Serial.print(","); Serial.print(g_mpu->accelZ());
  Serial.print(";G:"); Serial.print(g_mpu->gyroX());  Serial.print(","); Serial.print(g_mpu->gyroY());  Serial.print(","); Serial.print(g_mpu->gyroZ());
  Serial.print(";M:"); Serial.print(g_mpu->magX());   Serial.print(","); Serial.print(g_mpu->magY());   Serial.print(","); Serial.print(g_mpu->magZ());
  Serial.print(";T:"); if(isnan(tempC)) Serial.print("nan"); else Serial.print(tempC,2);
  Serial.println();

  // 🧼 aislar para no contaminar el bus antes de cambiar de canal
  mpu_quarantine(g_mpuAddr);
  tcadeselectAll();
}

// ======================= BNO055 (canal 0) =======================
static bool bno_wait_fusion(uint8_t addr, uint16_t to_ms=400){
  uint8_t st=0; unsigned long t0=millis();
  while (millis()-t0 < to_ms){
    if (rd8(addr, 0x39, st) && st==0x05) return true; // SYS_STAT==0x05
    delay(5);
  }
  return false;
}

void read_bno_on_channel(uint8_t ch) {
  if (ch!=0){ Serial.print("BNO,"); Serial.print(ch); Serial.println(",NA"); return; }

  // Aislar MPU antes de tocar BNO
  tcaselect(2);
  if (!g_mpuAddr) detect_mpu(g_mpuAddr);
  if (g_mpuAddr)  mpu_quarantine(g_mpuAddr);
  tcadeselectAll();

  // Canal del BNO
  tcaselect(0);

  if (!g_bnoAddr && !detect_bno(g_bnoAddr)){
    Serial.println("BNO,0,NA"); tcadeselectAll(); return;
  }
  if (!g_bno){
    g_bno = new Adafruit_BNO055(55, g_bnoAddr, &Wire);
  }
  if (!g_bno_inited){
    if(!g_bno->begin()){ Serial.println("BNO,0,NA"); tcadeselectAll(); return; }
    g_bno->setExtCrystalUse(true);
    g_bno->setMode(OPERATION_MODE_NDOF);
    bno_wait_fusion(g_bnoAddr, 600);
    g_bno_inited = true;
  } else if (g_bno->getMode()!=OPERATION_MODE_NDOF){
    g_bno->setMode(OPERATION_MODE_NDOF);
    bno_wait_fusion(g_bnoAddr, 300);
  }

  imu::Vector<3> e = g_bno->getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> a = g_bno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> g = g_bno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  Serial.print("BNO,0,E:"); Serial.print(e.x()); Serial.print(','); Serial.print(e.y()); Serial.print(','); Serial.print(e.z());
  Serial.print(",A:");      Serial.print(a.x()); Serial.print(','); Serial.print(a.y()); Serial.print(','); Serial.print(a.z());
  Serial.print(",G:");      Serial.print(g.x()); Serial.print(','); Serial.print(g.y()); Serial.print(','); Serial.println(g.z());

  tcadeselectAll();
}

// ======================= INIT =======================
void sensores_init(){
  Wire.begin(); Wire.setClock(I2C_KHZ);
  tcadeselectAll();
  Serial.println("Sensores listos. (BNO ch0, MPU ch2 + quarantine)");
}

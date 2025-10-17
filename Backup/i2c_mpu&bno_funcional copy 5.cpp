#include <mpu9250&bno055.h>

#ifndef I2C_KHZ
#define I2C_KHZ 400000UL
#endif

// ---- DEBUG FLAG ----
#define SENSOR_DEBUG 0  // Cambiar a 0 para deshabilitar debug

#if SENSOR_DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINT_HEX(x) Serial.print(x, HEX)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT_HEX(x)
#endif

// ---- TCA ----
void tcaselect(uint8_t i){
  if(i>7) return;
  DEBUG_PRINT("[TCA sel "); DEBUG_PRINT(i); DEBUG_PRINTLN("]");
  Wire.beginTransmission(TCAADDR); Wire.write(1<<i); Wire.endTransmission();
  delayMicroseconds(500);
}
void tcadeselectAll(){
  DEBUG_PRINTLN("[TCA deselect all]");
  Wire.beginTransmission(TCAADDR); Wire.write(0x00); Wire.endTransmission();
}

// ---- I2C helpers ----
bool rd8(uint8_t a,uint8_t r,uint8_t& v){
  Wire.beginTransmission(a); Wire.write(r);
  if(Wire.endTransmission(false)!=0) {
    DEBUG_PRINT("[rd8 fail addr=0x"); DEBUG_PRINT_HEX(a); DEBUG_PRINTLN("]");
    return false;
  }
  if(Wire.requestFrom((int)a,1)!=1) {
    DEBUG_PRINT("[rd8 req fail addr=0x"); DEBUG_PRINT_HEX(a); DEBUG_PRINTLN("]");
    return false;
  }
  v=Wire.read(); 
  return true;
}
static inline bool rd16(uint8_t a,uint8_t r,int16_t& v){
  uint8_t msb, lsb;
  if(!rd8(a,r,msb)) return false;
  if(!rd8(a,(uint8_t)(r+1),lsb)) return false;
  v = (int16_t)((msb<<8) | lsb);
  return true;
}
bool wr8(uint8_t a,uint8_t r,uint8_t v){
  Wire.beginTransmission(a); Wire.write(r); Wire.write(v);
  bool ok = Wire.endTransmission(true)==0;
  if(!ok){
    DEBUG_PRINT("[wr8 fail addr=0x"); DEBUG_PRINT_HEX(a); DEBUG_PRINT(" reg=0x"); DEBUG_PRINT_HEX(r); DEBUG_PRINTLN("]");
  }
  return ok;
}

// ---- MPU quarantine ----
void mpu_quarantine(uint8_t a){
  DEBUG_PRINTLN("[MPU quarantine start]");
  wr8(a,0x6A,0x00); // USER_CTRL: I2C master off
  wr8(a,0x37,0x00); // INT_PIN_CFG: bypass off
  wr8(a,0x6B,0x40); // PWR_MGMT_1: sleep on
  delayMicroseconds(50);
  DEBUG_PRINTLN("[MPU quarantine end]");
}

// Despierta y habilita bypass AK8963 (para mag) en cada ciclo
static inline void mpu_wake_and_bypass(uint8_t a){
  DEBUG_PRINTLN("[MPU wake+bypass start]");
  wr8(a, 0x6B, 0x01); // PWR_MGMT_1: wake, clock auto
  wr8(a, 0x6A, 0x00); // USER_CTRL: I2C master OFF (necesario para bypass)
  wr8(a, 0x37, 0x02); // INT_PIN_CFG: I2C_BYPASS_EN=1
  delayMicroseconds(50);
  DEBUG_PRINTLN("[MPU wake+bypass end]");
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
  DEBUG_PRINTLN("[Detecting MPU...]");
  uint8_t who=0;
  if(rd8(0x68,0x75,who) && (who==0x71||who==0x73)){ 
    addr=0x68; 
    DEBUG_PRINTLN("[MPU found at 0x68]");
    return true; 
  }
  if(rd8(0x69,0x75,who) && (who==0x71||who==0x73)){ 
    addr=0x69; 
    DEBUG_PRINTLN("[MPU found at 0x69]");
    return true; 
  }
  DEBUG_PRINTLN("[MPU not found]");
  return false;
}
static bool detect_bno(uint8_t &addr){
  DEBUG_PRINTLN("[Detecting BNO...]");
  uint8_t id=0;
  if(rd8(0x29,0x00,id) && id==0xA0){ 
    addr=0x29; 
    DEBUG_PRINTLN("[BNO found at 0x29]");
    return true; 
  }
  if(rd8(0x28,0x00,id) && id==0xA0){ 
    addr=0x28; 
    DEBUG_PRINTLN("[BNO found at 0x28]");
    return true; 
  }
  DEBUG_PRINTLN("[BNO not found]");
  return false;
}

// ---- Temp MPU9250 en °C (reg 0x41-0x42)
static bool mpu_read_tempC(uint8_t addr, float &tempC){
  DEBUG_PRINTLN("[Reading MPU temp]");
  int16_t raw;
  if(!rd16(addr, 0x41, raw)) return false;
  tempC = (float)raw / 333.87f + 21.0f; // especificación MPU9250
  return true;
}

// ======================= MPU9250 (canal 2) =======================
void read_mpu_on_channel(uint8_t ch) {
  DEBUG_PRINT("[MPU read start ch="); DEBUG_PRINT(ch); DEBUG_PRINTLN("]");
  
  if (ch!=2){ 
    Serial.print("MPU,"); Serial.print(ch); Serial.println(",NA"); 
    return; 
  }

  tcaselect(2);
  DEBUG_PRINTLN("[MPU: TCA selected]");

  if (!g_mpuAddr && !detect_mpu(g_mpuAddr)){
    Serial.println("MPU,2,NA"); 
    tcadeselectAll(); 
    return;
  }
  
  DEBUG_PRINTLN("[MPU: Detected]");
  
  if (!g_mpu){
    DEBUG_PRINTLN("[MPU: Creating object]");
    g_mpu = new MPU9250_asukiaaa(g_mpuAddr);
    g_mpu->setWire(&Wire);
  }
  
  if (!g_mpu_inited){
    DEBUG_PRINTLN("[MPU: Init start]");
    // init una sola vez (sin quarantine aquí)
    wr8(g_mpuAddr,0x6B,0x01);                  // wake
    DEBUG_PRINTLN("[MPU: beginAccel]");
    g_mpu->beginAccel(); 
    DEBUG_PRINTLN("[MPU: beginGyro]");
    g_mpu->beginGyro(); 
    DEBUG_PRINTLN("[MPU: beginMag]");
    g_mpu->beginMag();
    g_mpu_inited = true;
    DEBUG_PRINTLN("[MPU: Init complete]");
  }

  // 🔧 clave: salir de sleep y habilitar bypass cada ciclo
  mpu_wake_and_bypass(g_mpuAddr);

  DEBUG_PRINTLN("[MPU: accelUpdate]");
  g_mpu->accelUpdate();
  DEBUG_PRINTLN("[MPU: gyroUpdate]");
  g_mpu->gyroUpdate();
  DEBUG_PRINTLN("[MPU: magUpdate]");
  g_mpu->magUpdate();

  float tempC = NAN;
  mpu_read_tempC(g_mpuAddr, tempC);
  DEBUG_PRINTLN("[MPU: Temp read]");

  // ==== FORMATO ORIGINAL (ROS) ====
  DEBUG_PRINTLN("[MPU: Printing data]");
  Serial.print("A:"); Serial.print(g_mpu->accelX()); Serial.print(","); Serial.print(g_mpu->accelY()); Serial.print(","); Serial.print(g_mpu->accelZ());
  Serial.print(";G:"); Serial.print(g_mpu->gyroX());  Serial.print(","); Serial.print(g_mpu->gyroY());  Serial.print(","); Serial.print(g_mpu->gyroZ());
  Serial.print(";M:"); Serial.print(g_mpu->magX());   Serial.print(","); Serial.print(g_mpu->magY());   Serial.print(","); Serial.print(g_mpu->magZ());
  Serial.print(";T:"); if(isnan(tempC)) Serial.print("nan"); else Serial.print(tempC,2);
  Serial.println();
  DEBUG_PRINTLN("[MPU: Data printed]");

  // 🧼 aislar para no contaminar el bus antes de cambiar de canal
  mpu_quarantine(g_mpuAddr);
  tcadeselectAll();
  DEBUG_PRINTLN("[MPU read complete]");
}

// ======================= BNO055 (canal 0) =======================
static bool bno_wait_fusion(uint8_t addr, uint16_t to_ms=1000){
  DEBUG_PRINTLN("[BNO wait_fusion start]");
  uint8_t st=0; 
  unsigned long t0=millis();
  uint16_t attempts = 0;
  while (millis()-t0 < to_ms && attempts < 200){
    attempts++;
    if (rd8(addr, 0x39, st)){
      DEBUG_PRINT("[BNO SYS_STAT=0x"); DEBUG_PRINT_HEX(st); DEBUG_PRINTLN("]");
      if(st==0x05) {
        DEBUG_PRINTLN("[BNO fusion OK]");
        return true;
      }
    }
    delay(5);
    // Watchdog: imprimir cada 200ms
    if(attempts % 40 == 0){
      DEBUG_PRINT(".");
    }
  }
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("[BNO fusion timeout]");
  return false;
}

void read_bno_on_channel(uint8_t ch) {
  DEBUG_PRINT("[BNO read start ch="); DEBUG_PRINT(ch); DEBUG_PRINTLN("]");
  
  if (ch!=0){ 
    Serial.print("BNO,"); Serial.print(ch); Serial.println(",NA"); 
    return; 
  }

  // Aislar MPU antes de tocar BNO
  DEBUG_PRINTLN("[BNO: Isolating MPU]");
  tcaselect(2);
  if (!g_mpuAddr) detect_mpu(g_mpuAddr);
  if (g_mpuAddr){
    DEBUG_PRINTLN("[BNO: Putting MPU to sleep]");
    wr8(g_mpuAddr, 0x6A, 0x00); // USER_CTRL off
    wr8(g_mpuAddr, 0x37, 0x00); // BYPASS off
    wr8(g_mpuAddr, 0x6B, 0x40); // SLEEP on
  }
  tcadeselectAll();
  DEBUG_PRINTLN("[BNO: MPU isolated, waiting 5ms]");
  delay(5);

  // Canal del BNO
  tcaselect(0);
  DEBUG_PRINTLN("[BNO: TCA selected, waiting 2ms]");
  delay(2);

  if (!g_bnoAddr && !detect_bno(g_bnoAddr)){
    Serial.println("BNO,0,NA"); 
    tcadeselectAll(); 
    return;
  }
  
  DEBUG_PRINTLN("[BNO: Detected]");
  
  if (!g_bno){
    DEBUG_PRINTLN("[BNO: Creating object]");
    g_bno = new Adafruit_BNO055(55, g_bnoAddr, &Wire);
  }
  
  if (!g_bno_inited){
    DEBUG_PRINT("BNO Init...");
    if(!g_bno->begin()){ 
      Serial.println("BNO,0,INIT_FAIL"); 
      tcadeselectAll(); 
      return; 
    }
    DEBUG_PRINTLN("[BNO: begin() OK]");
    DEBUG_PRINT("crystal...");
    g_bno->setExtCrystalUse(true);
    DEBUG_PRINTLN("[BNO: crystal set, waiting 20ms]");
    delay(20);
    DEBUG_PRINT("mode...");
    g_bno->setMode(OPERATION_MODE_NDOF);
    DEBUG_PRINTLN("[BNO: mode set]");
    DEBUG_PRINT("fusion...");
    bool fusion_ok = bno_wait_fusion(g_bnoAddr, 1000);
    if(!fusion_ok){
      Serial.println("BNO,0,FUSION_TIMEOUT");
    }
    DEBUG_PRINTLN("[BNO: Waiting extra 500ms]");
    delay(500);
    DEBUG_PRINTLN("OK");
    g_bno_inited = true;
  } else {
    DEBUG_PRINTLN("[BNO: Already initialized, checking mode]");
    uint8_t current_mode = g_bno->getMode();
    DEBUG_PRINT("[BNO: current_mode=0x"); DEBUG_PRINT_HEX(current_mode); DEBUG_PRINTLN("]");
    if(current_mode != OPERATION_MODE_NDOF){
      DEBUG_PRINTLN("[BNO: Re-setting mode]");
      g_bno->setMode(OPERATION_MODE_NDOF);
      bno_wait_fusion(g_bnoAddr, 100);
    }
  }

  DEBUG_PRINTLN("[BNO: Reading Euler]");
  imu::Vector<3> e = g_bno->getVector(Adafruit_BNO055::VECTOR_EULER);
  DEBUG_PRINTLN("[BNO: Reading Accel]");
  imu::Vector<3> a = g_bno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  DEBUG_PRINTLN("[BNO: Reading Gyro]");
  imu::Vector<3> g = g_bno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  DEBUG_PRINTLN("[BNO: Printing data]");
  Serial.print("BNO,0,E:"); Serial.print(e.x()); Serial.print(','); Serial.print(e.y()); Serial.print(','); Serial.print(e.z());
  Serial.print(",A:");      Serial.print(a.x()); Serial.print(','); Serial.print(a.y()); Serial.print(','); Serial.print(a.z());
  Serial.print(",G:");      Serial.print(g.x()); Serial.print(','); Serial.print(g.y()); Serial.print(','); Serial.print(g.z());
  Serial.println();
  DEBUG_PRINTLN("[BNO: Data printed]");

  tcadeselectAll();
  DEBUG_PRINTLN("[BNO read complete]");
}

// ======================= INIT =======================
void sensores_init(){
  Wire.begin(); 
  Wire.setClock(I2C_KHZ);
  tcadeselectAll();
  Serial.println("Sensores listos. (BNO ch0, MPU ch2 + quarantine)");
  #if SENSOR_DEBUG
  Serial.println("[DEBUG MODE ENABLED]");
  #endif
}
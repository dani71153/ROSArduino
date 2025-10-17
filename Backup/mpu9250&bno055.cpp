#include <mpu9250&bno055.h>

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

#if defined(WIRE_HAS_TIMEOUT) || defined(__AVR__) || defined(TWBR)
static inline void wire_set_timeout(uint32_t timeout, bool reset_with_timeout){
  Wire.setWireTimeout(timeout, reset_with_timeout);
}
static inline bool wire_timeout_flag(){
  return Wire.getWireTimeoutFlag();
}
static inline void wire_clear_timeout_flag(){
  Wire.clearWireTimeoutFlag();
}
#else
static inline void wire_set_timeout(uint32_t timeout, bool reset_with_timeout){
  (void)timeout;
  (void)reset_with_timeout;
}
static inline bool wire_timeout_flag(){
  return false;
}
static inline void wire_clear_timeout_flag(){
}
#endif

#define SERIAL_IF_PRINT(flag, x) do { if (flag) Serial.print(x); } while(0)
#define SERIAL_IF_PRINTLN(flag, x) do { if (flag) Serial.println(x); } while(0)
#define SERIAL_IF_PRINTLN_EMPTY(flag) do { if (flag) Serial.println(); } while(0)

static const uint32_t kWireClockFast = 400000UL;
static const uint32_t kWireClockSafe = 100000UL;
static uint32_t g_wire_clock_hz = kWireClockFast;
static uint8_t g_wire_timeout_streak = 0;

static inline void wire_set_clock(uint32_t hz){
  Wire.setClock(hz);
  g_wire_clock_hz = hz;
}

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

static inline bool wire_had_timeout(){
  bool flag = wire_timeout_flag();
  if(flag){
    DEBUG_PRINTLN("[Wire timeout detected]");
    wire_clear_timeout_flag();
    if (g_wire_timeout_streak < 255) {
      g_wire_timeout_streak++;
    }
    if (g_wire_clock_hz != kWireClockSafe && g_wire_timeout_streak >= 3){
      DEBUG_PRINTLN("[Wire clock fallback -> 100kHz]");
      wire_set_clock(kWireClockSafe);
      g_wire_timeout_streak = 0;
    }
  } else if (g_wire_timeout_streak){
    g_wire_timeout_streak--;
    if (g_wire_clock_hz != kWireClockFast && g_wire_timeout_streak == 0){
      DEBUG_PRINTLN("[Wire clock restored -> 400kHz]");
      wire_set_clock(kWireClockFast);
    }
  }
  return flag;
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
static inline bool mpu_wake_and_bypass(uint8_t a){
  DEBUG_PRINTLN("[MPU wake+bypass start]");
  bool ok = true;
  ok &= wr8(a, 0x6B, 0x01); // PWR_MGMT_1: wake, clock auto
  ok &= wr8(a, 0x6A, 0x00); // USER_CTRL: I2C master OFF (necesario para bypass)
  ok &= wr8(a, 0x37, 0x02); // INT_PIN_CFG: I2C_BYPASS_EN=1
  delayMicroseconds(50);
  if(!ok){
    DEBUG_PRINTLN("[MPU wake+bypass fail]");
  } else {
    DEBUG_PRINTLN("[MPU wake+bypass end]");
  }
  return ok;
}

// ---- Caches mínimas ----
static uint8_t  g_mpuAddr = 0;
static MPU9250_asukiaaa* g_mpu = nullptr;
static bool     g_mpu_inited = false;

static uint8_t  g_bnoAddr = 0;
static Adafruit_BNO055* g_bno = nullptr;
static bool     g_bno_inited = false;
static bool     g_bno_waiting_fusion = false;
static unsigned long g_bno_fusion_deadline = 0;
static unsigned long g_bno_last_init_attempt = 0;

static inline void bno_schedule_reinit(unsigned long now){
  g_bno_inited = false;
  g_bno_waiting_fusion = false;
  g_bno_fusion_deadline = 0;
  g_bno_last_init_attempt = now;
}

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
bool read_mpu_on_channel(uint8_t ch, MpuReading* out, bool emitSerial) {
  if(out){
    out->valid = false;
    out->ax = out->ay = out->az = 0.0f;
    out->gx = out->gy = out->gz = 0.0f;
    out->mx = out->my = out->mz = 0.0f;
    out->tempC = NAN;
  }

  DEBUG_PRINT("[MPU read start ch="); DEBUG_PRINT(ch); DEBUG_PRINTLN("]");
  
  if (ch!=2){ 
    SERIAL_IF_PRINT(emitSerial, "MPU,"); SERIAL_IF_PRINT(emitSerial, ch); SERIAL_IF_PRINTLN(emitSerial, ",NA"); 
    return false; 
  }

  wire_clear_timeout_flag();
  tcaselect(2);
  DEBUG_PRINTLN("[MPU: TCA selected]");
  if(wire_had_timeout()){
    SERIAL_IF_PRINTLN(emitSerial, "MPU,2,ERR");
    g_mpu_inited = false;
    if (g_mpuAddr) {
      mpu_quarantine(g_mpuAddr);
    }
    tcadeselectAll();
    return false;
  }

  if (!g_mpuAddr && !detect_mpu(g_mpuAddr)){
    SERIAL_IF_PRINTLN(emitSerial, "MPU,2,NA"); 
    tcadeselectAll(); 
    return false;
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
  if (!mpu_wake_and_bypass(g_mpuAddr) || wire_had_timeout()){
    SERIAL_IF_PRINTLN(emitSerial, "MPU,2,ERR");
    g_mpu_inited = false;
    mpu_quarantine(g_mpuAddr);
    tcadeselectAll();
    return false;
  }

  DEBUG_PRINTLN("[MPU: accelUpdate]");
  uint8_t accel_status = g_mpu->accelUpdate();
  DEBUG_PRINTLN("[MPU: gyroUpdate]");
  uint8_t gyro_status  = g_mpu->gyroUpdate();
  DEBUG_PRINTLN("[MPU: magUpdate]");
  uint8_t mag_status   = g_mpu->magUpdate();
  bool updates_timeout = wire_had_timeout();
  DEBUG_PRINTLN("[MPU: Validating update status]");
  if(accel_status != 0 || gyro_status != 0 || mag_status != 0 || updates_timeout){
    DEBUG_PRINT("[MPU: update status accel/gyro/mag = ");
    DEBUG_PRINT(accel_status); DEBUG_PRINT(",");
    DEBUG_PRINT(gyro_status); DEBUG_PRINT(",");
    DEBUG_PRINT(mag_status); DEBUG_PRINTLN("]");
    SERIAL_IF_PRINTLN(emitSerial, "MPU,2,ERR");
    g_mpu_inited = false;
    mpu_quarantine(g_mpuAddr);
    tcadeselectAll();
    return false;
  }

  float tempC = NAN;
  bool temp_ok = mpu_read_tempC(g_mpuAddr, tempC);
  bool temp_timeout = wire_had_timeout();
  if(!temp_ok || temp_timeout){
    DEBUG_PRINTLN("[MPU: Temp read failed]");
    SERIAL_IF_PRINTLN(emitSerial, "MPU,2,ERR");
    g_mpu_inited = false;
    mpu_quarantine(g_mpuAddr);
    tcadeselectAll();
    return false;
  }
  DEBUG_PRINTLN("[MPU: Temp read]");

  const float ax = g_mpu->accelX();
  const float ay = g_mpu->accelY();
  const float az = g_mpu->accelZ();
  const float gx = g_mpu->gyroX();
  const float gy = g_mpu->gyroY();
  const float gz = g_mpu->gyroZ();
  const float mx = g_mpu->magX();
  const float my = g_mpu->magY();
  const float mz = g_mpu->magZ();

  if(out){
    out->valid = true;
    out->ax = ax; out->ay = ay; out->az = az;
    out->gx = gx; out->gy = gy; out->gz = gz;
    out->mx = mx; out->my = my; out->mz = mz;
    out->tempC = tempC;
  }

  if (emitSerial) {
    DEBUG_PRINTLN("[MPU: Printing data]");
    Serial.print("A:"); Serial.print(ax); Serial.print(","); Serial.print(ay); Serial.print(","); Serial.print(az);
    Serial.print(";G:"); Serial.print(gx);  Serial.print(","); Serial.print(gy);  Serial.print(","); Serial.print(gz);
    Serial.print(";M:"); Serial.print(mx);   Serial.print(","); Serial.print(my);   Serial.print(mz);
    Serial.print(";T:"); if(isnan(tempC)) Serial.print("nan"); else Serial.print(tempC,2);
    Serial.println();
    DEBUG_PRINTLN("[MPU: Data printed]");
  }

  // 🧼 aislar para no contaminar el bus antes de cambiar de canal
  mpu_quarantine(g_mpuAddr);
  tcadeselectAll();
  DEBUG_PRINTLN("[MPU read complete]");
  return true;
}

// ======================= BNO055 (canal 0) =======================
bool read_bno_on_channel(uint8_t ch, BnoReading* out, bool emitSerial) {
  if(out){
    out->valid = false;
    out->eulerX = out->eulerY = out->eulerZ = 0.0f;
    out->accelX = out->accelY = out->accelZ = 0.0f;
    out->gyroX = out->gyroY = out->gyroZ = 0.0f;
  }

  DEBUG_PRINT("[BNO read start ch="); DEBUG_PRINT(ch); DEBUG_PRINTLN("]");
  
  if (ch!=0){ 
    SERIAL_IF_PRINT(emitSerial, "BNO,"); SERIAL_IF_PRINT(emitSerial, ch); SERIAL_IF_PRINTLN(emitSerial, ",NA"); 
    return false; 
  }

  wire_clear_timeout_flag();
  unsigned long now = millis();

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
  if(wire_had_timeout()){
    SERIAL_IF_PRINTLN(emitSerial, "BNO,0,TIMEOUT");
    bno_schedule_reinit(now);
    return false;
  }
  DEBUG_PRINTLN("[BNO: MPU isolated, waiting 0.8ms]");
  delayMicroseconds(800);

  // Canal del BNO
  tcaselect(0);
  DEBUG_PRINTLN("[BNO: TCA selected, waiting 0.2ms]");
  delayMicroseconds(200);

  if (!g_bnoAddr && !detect_bno(g_bnoAddr)){
    SERIAL_IF_PRINTLN(emitSerial, "BNO,0,NA"); 
    tcadeselectAll(); 
    return false;
  }
  
  DEBUG_PRINTLN("[BNO: Detected]");
  
  if (!g_bno){
    DEBUG_PRINTLN("[BNO: Creating object]");
    g_bno = new Adafruit_BNO055(55, g_bnoAddr, &Wire);
  }
  
  if (!g_bno_inited){
    if(g_bno_last_init_attempt != 0 && (now - g_bno_last_init_attempt) < 2000){
      SERIAL_IF_PRINTLN(emitSerial, "BNO,0,INIT_PENDING");
      tcadeselectAll();
      return false;
    }
    g_bno_last_init_attempt = now;
    DEBUG_PRINT("BNO Init...");
    if(!g_bno->begin()){ 
      SERIAL_IF_PRINTLN(emitSerial, "BNO,0,INIT_FAIL"); 
      bno_schedule_reinit(now);
      tcadeselectAll(); 
      return false; 
    }
    if(wire_had_timeout()){
      SERIAL_IF_PRINTLN(emitSerial, "BNO,0,TIMEOUT");
      bno_schedule_reinit(millis());
      tcadeselectAll();
      return false;
    }
    DEBUG_PRINTLN("[BNO: begin() OK]");
    DEBUG_PRINT("crystal...");
    g_bno->setExtCrystalUse(true);
    if(wire_had_timeout()){
      SERIAL_IF_PRINTLN(emitSerial, "BNO,0,TIMEOUT");
      bno_schedule_reinit(millis());
      tcadeselectAll();
      return false;
    }
    DEBUG_PRINTLN("[BNO: crystal set, waiting 20ms]");
    delay(20);
    DEBUG_PRINT("mode...");
    g_bno->setMode(OPERATION_MODE_NDOF);
    if(wire_had_timeout()){
      SERIAL_IF_PRINTLN(emitSerial, "BNO,0,TIMEOUT");
      bno_schedule_reinit(millis());
      tcadeselectAll();
      return false;
    }
    DEBUG_PRINTLN("[BNO: mode set]");
    g_bno_inited = true;
    g_bno_waiting_fusion = true;
    g_bno_fusion_deadline = millis() + 1000;
    DEBUG_PRINTLN("[BNO: fusion pending]");
    SERIAL_IF_PRINTLN(emitSerial, "BNO,0,FUSION_PENDING");
    tcadeselectAll();
    return false;
  } else {
    if(g_bno_waiting_fusion){
      DEBUG_PRINTLN("[BNO: Checking fusion status]");
      uint8_t st = 0;
      if(rd8(g_bnoAddr, 0x39, st) && st == 0x05){
        DEBUG_PRINTLN("[BNO: fusion ready]");
        g_bno_waiting_fusion = false;
      } else {
        if(wire_had_timeout()){
          SERIAL_IF_PRINTLN(emitSerial, "BNO,0,TIMEOUT");
          bno_schedule_reinit(millis());
          tcadeselectAll();
          return false;
        }
        if(millis() > g_bno_fusion_deadline){
          DEBUG_PRINTLN("[BNO: fusion deadline reached]");
          SERIAL_IF_PRINTLN(emitSerial, "BNO,0,FUSION_TIMEOUT");
          bno_schedule_reinit(millis());
          tcadeselectAll();
          return false;
        }
        SERIAL_IF_PRINTLN(emitSerial, "BNO,0,FUSION_PENDING");
        tcadeselectAll();
        return false;
      }
    }
    DEBUG_PRINTLN("[BNO: Already initialized, checking mode]");
    uint8_t current_mode = g_bno->getMode();
    bool mode_timeout = wire_had_timeout();
    DEBUG_PRINT("[BNO: current_mode=0x"); DEBUG_PRINT_HEX(current_mode); DEBUG_PRINTLN("]");
    if(mode_timeout){
      SERIAL_IF_PRINTLN(emitSerial, "BNO,0,TIMEOUT");
      bno_schedule_reinit(millis());
      tcadeselectAll();
      return false;
    }
    if(current_mode != OPERATION_MODE_NDOF){
      DEBUG_PRINTLN("[BNO: Re-setting mode]");
      g_bno->setMode(OPERATION_MODE_NDOF);
      if(wire_had_timeout()){
        SERIAL_IF_PRINTLN(emitSerial, "BNO,0,TIMEOUT");
        bno_schedule_reinit(millis());
        tcadeselectAll();
        return false;
      }
      g_bno_waiting_fusion = true;
      g_bno_fusion_deadline = millis() + 200;
      SERIAL_IF_PRINTLN(emitSerial, "BNO,0,FUSION_PENDING");
      tcadeselectAll();
      return false;
    }
  }

  DEBUG_PRINTLN("[BNO: Reading Euler]");
  imu::Vector<3> e = g_bno->getVector(Adafruit_BNO055::VECTOR_EULER);
  DEBUG_PRINTLN("[BNO: Reading Accel]");
  imu::Vector<3> a = g_bno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  DEBUG_PRINTLN("[BNO: Reading Gyro]");
  imu::Vector<3> g = g_bno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  const float ex = e.x();
  const float ey = e.y();
  const float ez = e.z();
  const float ax_bno = a.x();
  const float ay_bno = a.y();
  const float az_bno = a.z();
  const float gx_bno = g.x();
  const float gy_bno = g.y();
  const float gz_bno = g.z();

  if(wire_had_timeout()){
    SERIAL_IF_PRINTLN(emitSerial, "BNO,0,TIMEOUT");
    bno_schedule_reinit(millis());
    tcadeselectAll();
    return false;
  }

  if(out){
    out->valid = true;
    out->eulerX = ex; out->eulerY = ey; out->eulerZ = ez;
    out->accelX = ax_bno; out->accelY = ay_bno; out->accelZ = az_bno;
    out->gyroX = gx_bno; out->gyroY = gy_bno; out->gyroZ = gz_bno;
  }

  if (emitSerial) {
    DEBUG_PRINTLN("[BNO: Printing data]");
    Serial.print("BNO,0,E:"); Serial.print(ex); Serial.print(','); Serial.print(ey); Serial.print(','); Serial.print(ez);
    Serial.print(",A:");      Serial.print(ax_bno); Serial.print(','); Serial.print(ay_bno); Serial.print(','); Serial.print(az_bno);
    Serial.print(",G:");      Serial.print(gx_bno); Serial.print(','); Serial.print(gy_bno); Serial.print(','); Serial.print(gz_bno);
    Serial.println();
    DEBUG_PRINTLN("[BNO: Data printed]");
  }

  tcadeselectAll();
  DEBUG_PRINTLN("[BNO read complete]");
  return true;
}
// ======================= INIT =======================
void sensores_init(){
  Wire.begin(); 
  wire_set_clock(kWireClockFast);
  g_wire_timeout_streak = 0;
  wire_set_timeout(25000, false);
  wire_clear_timeout_flag();
  tcadeselectAll();
  // Serial.println("Sensores listos. (BNO ch0, MPU ch2 + quarantine)");
  #if SENSOR_DEBUG
  Serial.println("[DEBUG MODE ENABLED]");
  #endif
}

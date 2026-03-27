#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BNO055.h>

#define TCAADDR 0x70
#define I2C_KHZ 100000
#define DO_READ_PASS true  // true: hace la pasada B con lecturas

// --- TCA selección mínima (como tu base) ---
void tcaselect(uint8_t i){
  if(i>7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1<<i);
  Wire.endTransmission(); // STOP
}

// --- R/W helpers ---
bool rd8(uint8_t addr,uint8_t reg,uint8_t& val){
  Wire.beginTransmission(addr); Wire.write(reg);
  if(Wire.endTransmission(false)!=0) return false;     // repeated start
  if(Wire.requestFrom((int)addr,1)!=1) return false;
  val=Wire.read(); return true;
}
bool wr8(uint8_t addr,uint8_t reg,uint8_t val){
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission(true)==0;
}

// --- Cuarentena MPU: limpia BYPASS/MASTER y sleep opcional ---
void mpu_quarantine(uint8_t a){
  wr8(a,0x6A,0x00); delay(2); // USER_CTRL: I2C_MST_EN=0
  wr8(a,0x37,0x00); delay(2); // INT_PIN_CFG: BYPASS_EN=0
  wr8(a,0x6B,0x40); delay(2); // PWR_MGMT_1: SLEEP
}

// --- Lectura única del MPU en este canal (A,G,M) ---
bool read_mpu_once_on_this_channel(){
  uint8_t who=0; uint8_t mpuAddr=0;
  if(rd8(0x68,0x75,who) && (who==0x71||who==0x73)) mpuAddr=0x68;
  else if(rd8(0x69,0x75,who) && (who==0x71||who==0x73)) mpuAddr=0x69;
  else return false;

  // Wake + init mínimo
  wr8(mpuAddr,0x6B,0x80); delay(120); // reset
  wr8(mpuAddr,0x6B,0x01); delay(15);  // clock PLL

  MPU9250_asukiaaa mpu(mpuAddr);
  mpu.setWire(&Wire);
  mpu.beginAccel(); mpu.beginGyro(); mpu.beginMag();
  delay(25);

  mpu.accelUpdate(); mpu.gyroUpdate(); mpu.magUpdate();
  Serial.print("   MPU A:"); Serial.print(mpu.accelX()); Serial.print(',');
  Serial.print(mpu.accelY()); Serial.print(','); Serial.print(mpu.accelZ());
  Serial.print(" G:"); Serial.print(mpu.gyroX());  Serial.print(',');
  Serial.print(mpu.gyroY());  Serial.print(',');   Serial.print(mpu.gyroZ());
  Serial.print(" M:"); Serial.print(mpu.magX());   Serial.print(',');
  Serial.print(mpu.magY());   Serial.print(',');   Serial.println(mpu.magZ());

  // Cuarentena ANTES de salir del canal
  mpu_quarantine(mpuAddr);
  return true;
}

// --- Lectura única del BNO en este canal (Euler/Accel/Gyro) ---
bool read_bno_once_on_this_channel(){
  uint8_t id=0; uint8_t addr=0;
  if(rd8(0x28,0x00,id) && id==0xA0) addr=0x28;
  else if(rd8(0x29,0x00,id) && id==0xA0) addr=0x29;
  else return false;

  Adafruit_BNO055 bno(55, addr, &Wire);
  if(!bno.begin()) return false;
  delay(25);

  // Config → NDOF para leer; luego volver a CONFIG para no dejar fusión corriendo
  bno.setMode(OPERATION_MODE_CONFIG); delay(20);
  // bno.setExtCrystalUse(true); // si usas cristal externo
  bno.setMode(OPERATION_MODE_NDOF);  delay(30);

  imu::Vector<3> eul = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  Serial.print("   BNO E:");
  Serial.print(eul.x());  Serial.print(',');
  Serial.print(eul.y());  Serial.print(',');
  Serial.print(eul.z());
  Serial.print(" A:");
  Serial.print(acc.x());  Serial.print(',');
  Serial.print(acc.y());  Serial.print(',');
  Serial.print(acc.z());
  Serial.print(" G:");
  Serial.print(gyr.x());  Serial.print(',');
  Serial.print(gyr.y());  Serial.print(',');
  Serial.println(gyr.z());

  // Volver a CONFIG (estado neutro)
  bno.setMode(OPERATION_MODE_CONFIG); delay(20);
  return true;
}

void setup(){
  while(!Serial);
  delay(1000);
  Wire.begin(); Wire.setClock(I2C_KHZ);

  Serial.begin(115200);
  Serial.println("\nTCA escáner con lectura de MPU9250 y BNO055");

  // PASADA A — probes de escritura (tu base)
  for(uint8_t ch=0; ch<8; ch++){
    tcaselect(ch);
    Serial.print("A) Escaneando salida "); Serial.println(ch);
    for(uint8_t addr=1; addr<127; addr++){
      if(addr==TCAADDR) continue;
      Wire.beginTransmission(addr);
      if(Wire.endTransmission()==0){
        Serial.print("  - Encontrado 0x"); Serial.println(addr,HEX);
      }
    }
  }

  if(!DO_READ_PASS){ Serial.println("Finalizado (solo probes)."); return; }

  // PASADA B — lectura mínima + lectura real (MPU y BNO)
  for(uint8_t ch=0; ch<8; ch++){
    tcaselect(ch);
    Serial.print("B) Test lectura en salida "); Serial.println(ch);

    uint8_t v=0; bool any=false;
    if(rd8(0x68,0x75,v) || rd8(0x69,0x75,v)){ any=true; Serial.print("   MPU WHOAMI=0x"); Serial.println(v,HEX); }
    if(rd8(0x28,0x00,v) || rd8(0x29,0x00,v)){ any=true; Serial.print("   BNO ID=0x");    Serial.println(v,HEX); }
    if(!any) Serial.println("   (sin lectura mínima válida)");

    // Lecturas reales (cada una deja el bus “limpio” al salir)
    (void)read_mpu_once_on_this_channel();
    (void)read_bno_once_on_this_channel();

    // STOP total y pequeño settle antes del próximo canal
    Wire.endTransmission(true);
    delay(10);
  }

  Serial.println("Finalizado.");
}

void loop(){}

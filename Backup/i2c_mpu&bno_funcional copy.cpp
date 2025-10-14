#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BNO055.h>

#define TCAADDR 0x70
#define I2C_KHZ 100000

// ---------- TCA ----------
void tcaselect(uint8_t i){
  if(i>7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
void tcadeselectAll(){
  Wire.beginTransmission(TCAADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}

// ---------- R/W helpers ----------
bool rd8(uint8_t addr,uint8_t reg,uint8_t& val){
  Wire.beginTransmission(addr); Wire.write(reg);
  if(Wire.endTransmission(false)!=0) return false;
  if(Wire.requestFrom((int)addr,1)!=1) return false;
  val = Wire.read(); return true;
}
bool wr8(uint8_t addr,uint8_t reg,uint8_t val){
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission(true)==0;
}

// ---------- “Cuarentena” MPU ----------
void mpu_quarantine(uint8_t a){
  wr8(a,0x6A,0x00); delay(2);
  wr8(a,0x37,0x00); delay(2);
  wr8(a,0x6B,0x40); delay(2);
}

// ---------- Lecturas ----------
bool read_mpu_once_on_this_channel(){
  uint8_t who=0, mpuAddr=0;
  if(rd8(0x68,0x75,who) && (who==0x71||who==0x73)) mpuAddr=0x68;
  else if(rd8(0x69,0x75,who) && (who==0x71||who==0x73)) mpuAddr=0x69;
  else return false;

  wr8(mpuAddr,0x6B,0x80); delay(120);
  wr8(mpuAddr,0x6B,0x01); delay(15);

  MPU9250_asukiaaa mpu(mpuAddr);
  mpu.setWire(&Wire);
  mpu.beginAccel(); mpu.beginGyro(); mpu.beginMag(); delay(25);
  mpu.accelUpdate(); mpu.gyroUpdate(); mpu.magUpdate();

  Serial.print("MPU A:"); Serial.print(mpu.accelX()); Serial.print(',');
  Serial.print(mpu.accelY()); Serial.print(','); Serial.print(mpu.accelZ());
  Serial.print(" G:"); Serial.print(mpu.gyroX());  Serial.print(',');
  Serial.print(mpu.gyroY());  Serial.print(',');   Serial.print(mpu.gyroZ());
  Serial.print(" M:"); Serial.print(mpu.magX());   Serial.print(',');
  Serial.print(mpu.magY());   Serial.print(',');   Serial.println(mpu.magZ());

  mpu_quarantine(mpuAddr);
  return true;
}

bool read_bno_once_on_this_channel(){
  uint8_t id=0, addr=0;
  if(rd8(0x28,0x00,id) && id==0xA0) addr=0x28;
  else if(rd8(0x29,0x00,id) && id==0xA0) addr=0x29;
  else return false;

  Adafruit_BNO055 bno(55, addr, &Wire);
  if(!bno.begin()) return false;
  delay(25);

  bno.setMode(OPERATION_MODE_CONFIG); delay(20);
  // bno.setExtCrystalUse(true);
  bno.setMode(OPERATION_MODE_NDOF);  delay(30);

  imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> g = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  Serial.print("BNO E:"); Serial.print(e.x()); Serial.print(',');
  Serial.print(e.y()); Serial.print(','); Serial.print(e.z());
  Serial.print(" A:");   Serial.print(a.x()); Serial.print(',');
  Serial.print(a.y());  Serial.print(',');   Serial.print(a.z());
  Serial.print(" G:");   Serial.print(g.x()); Serial.print(',');
  Serial.print(g.y());  Serial.print(',');   Serial.println(g.z());

  bno.setMode(OPERATION_MODE_CONFIG); delay(20);
  return true;
}

// ---------- Utilidad ----------
void read_auto_on_channel(uint8_t ch){
  tcaselect(ch);
  Serial.print("\nCanal "); Serial.println(ch);
  bool ok = read_mpu_once_on_this_channel();
  if(!ok) ok = read_bno_once_on_this_channel();
  if(!ok) Serial.println("(sin sensor válido en este canal)");
  tcadeselectAll();
}

int8_t ask_channel(){
  Serial.print("Canal TCA (0-7): "); Serial.flush();
  while(!Serial.available()){}
  int v = Serial.parseInt(); while(Serial.available()) Serial.read();
  if(v<0 || v>7) return -1;
  return (uint8_t)v;
}

// Lee la próxima tecla “real” (ignora \r y \n y espacios)
char getKey(){
  while(true){
    while(!Serial.available()){}
    char c = Serial.read();
    if(c!='\r' && c!='\n' && c!=' ') return c;
  }
}

void print_prompt(){
  Serial.println("\nOpciones:");
  Serial.println("  [1] Pedir canal y leer (auto MPU/BNO)");
  Serial.println("  [0-7] Leer directamente ese canal (p.ej. 2 = canal 2)");
  Serial.print("> ");
}

void setup(){
  Serial.begin(115200);
  delay(300);
  Wire.begin(); Wire.setClock(I2C_KHZ);
  tcadeselectAll();
  Serial.println("\nListo: lectura por canal (auto MPU/BNO).");
  print_prompt();
}

void loop(){
  char c = getKey();            // <-- robusto contra CR/LF
  if(c=='1'){
    int8_t ch = ask_channel();
    if(ch<0) Serial.println("Canal inválido.");
    else     read_auto_on_channel((uint8_t)ch);
  } else if(c>='0' && c<='7'){
    read_auto_on_channel((uint8_t)(c - '0'));  // ‘2’ => canal 2
  } else {
    Serial.println("Opción inválida.");
  }
  // limpia restos de la línea
  while(Serial.available()) Serial.read();
  print_prompt();
}

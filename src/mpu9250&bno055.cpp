// mpu9250_bno055.cpp
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <MPU9250_asukiaaa.h>

//==================== TCA + I2C Manager ====================
class TcaI2cBus {
public:
  TcaI2cBus(TwoWire& w, uint8_t tcaAddr, uint32_t hz=400000)
    : wire(w), TCA_ADDR(tcaAddr), i2cHz(hz), firstSelect(true) {}

  bool begin() {
    wire.begin();
    wire.setClock(i2cHz);
    delay(50);
    busClear();
    return true;
  }

  bool select(uint8_t ch, uint16_t settle_ms=2) {
    wire.setClock(i2cHz);
    wire.beginTransmission(TCA_ADDR);
    wire.write(uint8_t(1u << ch));
    if (wire.endTransmission()!=0) return false;
    if (firstSelect) { delay(10); firstSelect=false; }
    delay(settle_ms);
    return true;
  }

  void setClock(uint32_t hz){ i2cHz = hz; wire.setClock(i2cHz); }
  TwoWire& getWire(){ return wire; }

  void busClear(){
#if defined(SCL) && defined(SDA)
    pinMode(SCL, OUTPUT);
    pinMode(SDA, INPUT_PULLUP);
    for(int i=0;i<9;i++){
      digitalWrite(SCL,HIGH); delayMicroseconds(5);
      digitalWrite(SCL,LOW);  delayMicroseconds(5);
    }
    digitalWrite(SCL,HIGH);
    delayMicroseconds(5);
#endif
  }

private:
  TwoWire&  wire;
  const uint8_t TCA_ADDR;
  uint32_t i2cHz;
  bool firstSelect;
};

//==================== MPU-9250 Driver ====================
class Mpu9250Driver {
public:
  Mpu9250Driver(TcaI2cBus& bus, uint8_t tcaChannel, uint8_t addrGuess=0x68)
    : bus(bus), ch(tcaChannel), guess(addrGuess), cur(addrGuess), imu(nullptr),
      ax(0),ay(0),az(0),gx(0),gy(0),gz(0),mx(0),my(0),mz(0),tempC(NAN) {}

  ~Mpu9250Driver(){ if (imu) delete imu; }

  bool begin(){
    bus.select(ch,10);
    if (!detectAddress()) return false;

    // Reset + PLL
    writeReg8(0x6B,0x80); delay(100);
    writeReg8(0x6B,0x01); delay(10);

    if (imu) delete imu;
    imu = new MPU9250_asukiaaa(cur);
    imu->setWire(&bus.getWire());

    // init (funciones son void) + validación simple
    for (int k=0;k<3;k++){
      imu->beginAccel();
      imu->beginGyro();
      imu->beginMag();
      delay(20);

      uint8_t who=0;
      if (readReg8(cur,0x75,who) && (who==0x71 || who==0x73)) {
        imu->accelUpdate(); imu->gyroUpdate(); imu->magUpdate();
        return true;
      }
      writeReg8(0x6B,0x01); delay(10);
    }
    return false;
  }

  void update(){
    if (!imu) return;
    bus.select(ch,2);
    imu->accelUpdate();  ax=imu->accelX(); ay=imu->accelY(); az=imu->accelZ();
    imu->gyroUpdate();   gx=imu->gyroX();  gy=imu->gyroY();  gz=imu->gyroZ();
    imu->magUpdate();    mx=imu->magX();   my=imu->magY();   mz=imu->magZ();
    tempC = readTempC();
  }

  void printSerial(){
    Serial.print("A:"); Serial.print(ax); Serial.print(","); Serial.print(ay); Serial.print(","); Serial.print(az);
    Serial.print(";G:"); Serial.print(gx); Serial.print(","); Serial.print(gy); Serial.print(","); Serial.print(gz);
    Serial.print(";M:"); Serial.print(mx); Serial.print(","); Serial.print(my); Serial.print(","); Serial.print(mz);
    Serial.print(";T:"); Serial.print(tempC,2);
  }

private:
  TcaI2cBus& bus; const uint8_t ch, guess; uint8_t cur;
  MPU9250_asukiaaa* imu;
  float ax,ay,az,gx,gy,gz,mx,my,mz,tempC;

  bool readReg8(uint8_t addr,uint8_t reg,uint8_t& val){
    TwoWire& w = bus.getWire();
    bus.select(ch,2);
    w.beginTransmission(addr); w.write(reg);
    if (w.endTransmission(false)!=0) return false;
    if (w.requestFrom((int)addr,1)!=1) return false;
    val = w.read();
    return true;
  }
  bool writeReg8(uint8_t reg,uint8_t val){
    TwoWire& w = bus.getWire();
    bus.select(ch,2);
    w.beginTransmission(cur); w.write(reg); w.write(val);
    return w.endTransmission()==0;
  }
  bool detectAddress(){
    uint8_t who=0;
    if (readReg8(guess,0x75,who) && (who==0x71 || who==0x73)) { cur=guess; return true; }
    uint8_t alt = (guess==0x68)?0x69:0x68;
    if (readReg8(alt,  0x75,who) && (who==0x71 || who==0x73)) { cur=alt;  return true; }
    return false;
  }
  float readTempC(){
    uint8_t h=0,l=0;
    if (!readReg8(cur,0x41,h)) return NAN;
    if (!readReg8(cur,0x42,l)) return NAN;
    int16_t raw = (int16_t)((h<<8)|l);
    return (raw/333.87f)+21.0f;
  }
};

//==================== BNO055 Driver ====================
class Bno055Driver {
public:
  Bno055Driver(TcaI2cBus& bus,uint8_t tcaChannel,uint8_t addrGuess=0x28)
    : bus(bus), ch(tcaChannel), guess(addrGuess), cur(addrGuess), bno(nullptr),
      ax(0),ay(0),az(0),gx(0),gy(0),gz(0),mx(0),my(0),mz(0),
      yaw(0),pitch(0),roll(0),tempC(NAN), sys(0),gC(0),aC(0),mC(0) {}

  ~Bno055Driver(){ if (bno) delete bno; }

  bool begin(){
    bus.select(ch,10);
    if (!detectAddress()) return false;

    if (bno) delete bno;
    bno = new Adafruit_BNO055(55, cur, &bus.getWire());
    if (!bno->begin()) return false;
    delay(10);
    bno->setExtCrystalUse(true);
    delay(10);
    return true;
  }

  void update(){
    if (!bno) return;
    bus.select(ch,2);
    imu::Vector<3> acc = bno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyr = bno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mg  = bno->getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Vector<3> eul = bno->getVector(Adafruit_BNO055::VECTOR_EULER);
    ax=acc.x(); ay=acc.y(); az=acc.z();
    gx=gyr.x(); gy=gyr.y(); gz=gyr.z();
    mx=mg.x();  my=mg.y();  mz=mg.z();
    yaw=eul.x(); pitch=eul.y(); roll=eul.z();
    tempC=bno->getTemp();
    bno->getCalibration(&sys,&gC,&aC,&mC);
  }

  void printSerial(){
    Serial.print("A:"); Serial.print(ax,3); Serial.print(","); Serial.print(ay,3); Serial.print(","); Serial.print(az,3);
    Serial.print(";G:"); Serial.print(gx,3); Serial.print(","); Serial.print(gy,3); Serial.print(","); Serial.print(gz,3);
    Serial.print(";M:"); Serial.print(mx,3); Serial.print(","); Serial.print(my,3); Serial.print(","); Serial.print(mz,3);
    Serial.print(";E:"); Serial.print(yaw,2); Serial.print(","); Serial.print(pitch,2); Serial.print(","); Serial.print(roll,2);
    Serial.print(";T:"); Serial.print(tempC,1);
    Serial.print(";CAL:"); Serial.print(sys); Serial.print(","); Serial.print(gC); Serial.print(","); Serial.print(aC); Serial.print(","); Serial.print(mC);
  }

private:
  TcaI2cBus& bus; const uint8_t ch, guess; uint8_t cur;
  Adafruit_BNO055* bno;
  float ax,ay,az,gx,gy,gz,mx,my,mz,yaw,pitch,roll,tempC;
  uint8_t sys,gC,aC,mC;

  bool readReg8(uint8_t addr,uint8_t reg,uint8_t& val){
    TwoWire& w = bus.getWire();
    bus.select(ch,2);
    w.beginTransmission(addr); w.write(reg);
    if (w.endTransmission(false)!=0) return false;
    if (w.requestFrom((int)addr,1)!=1) return false;
    val = w.read();
    return true;
  }
  bool detectAddress(){
    uint8_t id=0;
    if (readReg8(guess,0x00,id) && id==0xA0){ cur=guess; return true; }
    uint8_t alt = (guess==0x28)?0x29:0x28;
    if (readReg8(alt,  0x00,id) && id==0xA0){ cur=alt;  return true; }
    return false;
  }
};

#include "mpu9250&bno055.h"

void setup() {
  Serial.begin(115200);
  sensores_init();
}

void loop() {
  // Lee el BNO en canal 0 cada segundo
  read_bno_on_channel(0);
  delay(1000);

  // Lee el MPU en canal 2 cada segundo
  read_mpu_on_channel(2);
  delay(1000);
}

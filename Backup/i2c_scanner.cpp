#include <Arduino.h>
#include <Wire.h>

// Dirección I2C del PCA9548A (por defecto es 0x70, verifica tu hardware)
#define PCA9548A_ADDR 0x70

// Selecciona el canal [0-7] del PCA9548A
void selectPCAChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel); // Bitmask: 1 en la posición del canal
  Wire.endTransmission();
  delay(2); // Pequeño retardo por seguridad
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);

  Serial.println("===== I2C Scanner con PCA9548A =====");
  Serial.print("Buscando PCA9548A en direccion 0x");
  Serial.println(PCA9548A_ADDR, HEX);

  // Detectar PCA9548A
  Wire.beginTransmission(PCA9548A_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println("PCA9548A detectado correctamente.");
  } else {
    Serial.println("ERROR: PCA9548A NO detectado. Verifica conexion y direccion.");
    while (1);
  }
}

void scanI2C(uint8_t channel) {
  Serial.print("Canal PCA9548A: ");
  Serial.println(channel);

  selectPCAChannel(channel);

  bool found = false;
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (addr == PCA9548A_ADDR) continue; // Ignorar el multiplexor

    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  I2C encontrado en direccion 0x");
      Serial.println(addr, HEX);
      found = true;
    }
    delay(2); // Retardo pequeño para estabilidad
  }
  if (!found) {
    Serial.println("  No se detectaron dispositivos.");
  }
  Serial.println();
}

void loop() {
  Serial.println("==== ESCANEO COMPLETO ====");
  for (uint8_t channel = 0; channel < 8; channel++) {
    scanI2C(channel);
  }
  Serial.println("==== FIN DEL ESCANEO ====\n");
  delay(5000); // Espera 5 segundos antes de volver a escanear
}

// --- PINES DE PRUEBA PARA EL MOTOR 1 ---
#define M1_RPWM 13
#include "MotorControlPIDV2_Arduino.cpp" // Se recomienda incluir el .h
// --- PINES DE PRUEBA PARA EL MOTOR 1 ---
#define M1_RPWM 13
#define M1_LPWM 11
#define M1_R_EN 12
#define M1_L_EN 10

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando Prueba de Fuego del Motor...");

  // Configurar pines como salida
  pinMode(M1_RPWM, OUTPUT);
  pinMode(M1_LPWM, OUTPUT);
  pinMode(M1_R_EN, OUTPUT);
  pinMode(M1_L_EN, OUTPUT);

  // ¡IMPORTANTE! Habilitar el driver BTS7960
  digitalWrite(M1_R_EN, HIGH);
  digitalWrite(M1_L_EN, HIGH);

  Serial.println("Driver Habilitado. El motor deberia moverse en 2 segundos...");
}

void loop() {
  // --- Mover hacia adelante (velocidad media) ---
  Serial.println("Moviendo hacia ADELANTE");
  analogWrite(M1_RPWM, 150); // PWM de 0 a 255
  analogWrite(M1_LPWM, 0);
  delay(3000); // Mover por 3 segundos

  // --- Frenar ---
  Serial.println("Frenando...");
  analogWrite(M1_RPWM, 0);
  analogWrite(M1_LPWM, 0);
  delay(2000); // Esperar 2 segundos

  // --- Mover hacia atrás (velocidad media) ---
  Serial.println("Moviendo hacia ATRAS");
  analogWrite(M1_RPWM, 0);
  analogWrite(M1_LPWM, 150); // PWM de 0 a 255
  delay(3000); // Mover por 3 segundos

  // --- Frenar ---
  Serial.println("Frenando...");
  analogWrite(M1_RPWM, 0);
  analogWrite(M1_LPWM, 0);
  delay(2000); // Esperar 2 segundos
}
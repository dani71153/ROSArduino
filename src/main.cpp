#include "MotorControlPIDV2_Arduino.cpp" // Se recomienda incluir el .h
#include <ACS712.h>

// === PINES Y CONFIGURACIÓN DE MOTORES (ACTUALIZADO PARA BTS7960) ===

// --- Pines Motor 1 (Derecho) ---
#define M1_RPWM 13
#define M1_LPWM 11
#define M1_R_EN 12
#define M1_L_EN 10
#define M1_ENC_A 18
#define M1_ENC_B 19

// --- Pines Motor 2 (Izquierdo) ---
#define M2_RPWM 4  // antes era 7
#define M2_LPWM 7  // antes era 4
#define M2_R_EN 6
#define M2_L_EN 5
#define M2_ENC_A 3
#define M2_ENC_B 2

// --- Instanciación de los objetos Motor para BTS7960 --- 
Motor motor1(M1_RPWM, M1_LPWM, M1_R_EN, M1_L_EN, M1_ENC_A, M1_ENC_B, 0.12, 0.0857, 0.001, 10); //Punto muy cecano 0.075 //PID calibrado, sin peso. 
Motor motor2(M2_RPWM, M2_LPWM, M2_R_EN, M2_L_EN, M2_ENC_A, M2_ENC_B, 0.12, 0.09, 0.001, 10);

// === CONFIGURACIÓN DEL SENSOR ACS712 ===
ACS712 myACS(A15, 5.0, 1023, 185);

// === VARIABLES DE CONTROL Y ESTADO ===
String inputCommand = ""; 
bool usarPID = true; 
unsigned long lastCommandTime = 0;
const unsigned long timeout = 3000;

// --> NUEVO: Variables para el control de posición
bool enControlDePosicion = false; // Bandera para saber si estamos en modo posición
const float PULSOS_POR_REVOLUCION = 270.0 * 64.0; // ¡Debe coincidir con el valor en la clase Motor!
long targetPosMotor1 = 0;
long targetPosMotor2 = 0;
const float Kp_posicion = 0.05; // Ganancia Proporcional para convertir error de posición a velocidad
const int POSICION_TOLERANCIA = 50; // Ticks de error para considerar que se ha llegado al objetivo

void processCommand(String command);

void setup() {
  Serial.begin(115200);
  Serial.println("Inicializando el Arduino Mega (Rebooting)");

  motor1.inicializar();
  motor2.inicializar();
  motor1.resetEncoderValues();
  motor2.resetEncoderValues();

  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH); 
  myACS.autoMidPointDC(1000);
  myACS.setNoisemV(50.88);
  
  lastCommandTime = millis();
}

void loop() {
  // Verificar si hay datos disponibles en el Serial
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    if (receivedChar == '<') {
      inputCommand = "";
    } else if (receivedChar == '>') {
      processCommand(inputCommand);
      inputCommand = "";
      lastCommandTime = millis();
    } else {
      inputCommand += receivedChar;
    }
  }

  // Actualizar los motores si se está usando PID
  if (usarPID) {
    // --> NUEVO: Lógica para el control de posición
    // Si la bandera de control de posición está activa, calculamos la velocidad necesaria
    // para llegar al objetivo. Esta velocidad se usará como referencia para el PID de velocidad.
    if (enControlDePosicion) {
      long currentPos1 = motor1.leerEncoder();
      long currentPos2 = motor2.leerEncoder();
      long errorPos1 = targetPosMotor1 - currentPos1;
      long errorPos2 = targetPosMotor2 - currentPos2;

      // Comprobar si hemos llegado al destino
      if (abs(errorPos1) < POSICION_TOLERANCIA && abs(errorPos2) < POSICION_TOLERANCIA) {
        motor1.setReferenciaVelocidad(0);
        motor2.setReferenciaVelocidad(0);
        enControlDePosicion = false; // Desactivamos el modo posición
        usarPID = false;             // Detenemos el PID
        Serial.println("<Posicion alcanzada>");
      } else {
        // Si no hemos llegado, calculamos la velocidad de referencia
        float velSetpoint1 = Kp_posicion * errorPos1;
        float velSetpoint2 = Kp_posicion * errorPos2;
        // Asignamos esta velocidad como el nuevo objetivo del PID de velocidad
        motor1.setReferenciaVelocidad(velSetpoint1);
        motor2.setReferenciaVelocidad(velSetpoint2);
      }
    }
    
    // Esta parte se ejecuta siempre que usarPID es true.
    // En modo velocidad, usa la referencia de 'm'.
    // En modo posición, usa la referencia calculada justo arriba.
    motor1.actualizar();
    motor2.actualizar();
  }

  // Verificar si ha pasado el tiempo de espera sin recibir comandos
  if (millis() - lastCommandTime > timeout) {
    motor1.controlarMotor(0);
    motor2.controlarMotor(0);
    motor1.desactivarMotor();
    motor2.desactivarMotor();
    usarPID = false;
    enControlDePosicion = false; // --> MODIFICADO: También resetea la bandera de posición
  }
}

void processCommand(String command) {
  if (command.length() == 0) return;

  switch (command[0]) {
    case 'm': { // Control de Velocidad
      // --> MODIFICADO: Si se recibe un comando de velocidad, se cancela el de posición
      enControlDePosicion = false; 

      if (command.length() < 3 || command[1] != ' ') {
        Serial.println("<Error: Formato invalido. Debe ser <m valor1 valor2>>");
        break;
      }
      command.remove(0, 2);
      int spaceIndex = command.indexOf(' ');
      if (spaceIndex == -1) { Serial.println("<Error: Formato invalido>"); break; }

      float velMotor1RPS = command.substring(0, spaceIndex).toFloat();
      float velMotor2RPS = command.substring(spaceIndex + 1).toFloat();
      motor1.setReferenciaVelocidadRPS(velMotor1RPS);
      motor2.setReferenciaVelocidadRPS(velMotor2RPS);
      motor1.actualizar();  // Ejecuta un ciclo completo con la nueva referencia
      motor2.actualizar();
      usarPID = true;
      motor1.sincronizarRampa();  // función que vamos a crear
      motor2.sincronizarRampa();


      if (velMotor1RPS == 0 && velMotor2RPS == 0) {
        motor1.controlarMotor(0);
        motor2.controlarMotor(0);
        usarPID = false;
        motor1.desactivarMotor();
        motor2.desactivarMotor();
      }
      break;
    }

    // --> NUEVO: Caso para el control de posición por vueltas
    case 'p': { 
      if (command.length() < 3 || command[1] != ' ') {
        Serial.println("<Error: Formato invalido. Debe ser <p vueltas1 vueltas2>>");
        break;
      }
      command.remove(0, 2);
      int spaceIndex = command.indexOf(' ');
      if (spaceIndex == -1) { Serial.println("<Error: Formato invalido>"); break; }

      float vueltasMotor1 = command.substring(0, spaceIndex).toFloat();
      float vueltasMotor2 = command.substring(spaceIndex + 1).toFloat();

      // Calcular la posición objetivo en ticks relativa a la posición actual
      targetPosMotor1 = motor1.leerEncoder() + (long)(vueltasMotor1 * PULSOS_POR_REVOLUCION);
      targetPosMotor2 = motor2.leerEncoder() + (long)(vueltasMotor2 * PULSOS_POR_REVOLUCION);

      Serial.print("<Moviendo a posicion (ticks): ");
      Serial.print(targetPosMotor1); Serial.print(", "); Serial.print(targetPosMotor2);
      Serial.println(">");

      // Activamos las banderas para iniciar el control de posición en el loop()
      enControlDePosicion = true;
      usarPID = true;
      break;
    }

    case 'o': {
      // --> MODIFICADO: Cancelar también el modo posición
      enControlDePosicion = false; 
      
      command.remove(0, 1);
      int spaceIndex = command.indexOf(' ');
      int pwmMotor1Value = command.substring(0, spaceIndex).toInt();
      int pwmMotor2Value = command.substring(spaceIndex + 1).toInt();
      motor1.controlarMotor(pwmMotor1Value);
      motor2.controlarMotor(pwmMotor2Value);
      usarPID = false;
      Serial.println("<Control PWM directo activado>");
      break;
    }

    case 'b': {
      Serial.print("<Baudrate actual: 115200>");
      break;
    }

    case 'e': {
      Serial.print("<");
      Serial.print(motor1.leerEncoder()); Serial.print(","); Serial.print(motor2.leerEncoder());
      Serial.println(">");
      break;
    }

    case 'r': {
      motor1.resetEncoderValues();
      motor2.resetEncoderValues();
      Serial.println("<Encoders reseteados>");
      break;
    }

    case 'i': {
      Serial.println("<OK>");
      break;
    }

    case 'v': {
      Serial.print("<");
      Serial.print(motor1.getVelocidadRPS()); Serial.print(","); Serial.print(motor2.getVelocidadRPS());
      Serial.println(">");
      break;
    }

    case 'c': {
      float current_mA = myACS.mA_DC(60);
      Serial.print("<");
      Serial.print(current_mA / 1000);
      Serial.println(">");
      break;
    }

    default: {
      Serial.println("<Comando invalido>");
      break;
    }
  }
}
#include <MotorControlPIDV1_Arduino.cpp>
#include <ACS712.h>

// === CONFIGURACIÓN DEL SENSOR ACS712 ===
ACS712 myACS(A15, 5.0, 1023, 185);

// Instanciar motores
Motor motor1(5, 7, 8, 19, 18, 0.1, 0, 0.015, 1);
Motor motor2(44, A13, A12, 3, 2, 0.1, 0.0, 0.015, 1);

String inputCommand = ""; // Variable para almacenar el comando recibido
void processCommand(String command);
bool usarPID = true; // Variable para controlar si se usa PID o no
unsigned long lastCommandTime = 0; // Variable para almacenar el tiempo del último comando recibido
const unsigned long timeout = 3000; // Tiempo de espera (3 segundos)

void setup() {
  Serial.begin(115200);
  Serial.println("Inicializando el Arduino Mega (Rebooting)");

  // Inicializar motores
  motor1.inicializar();
  motor2.inicializar();

  // Inicializar el VCC del ACS712
  pinMode(53, INPUT_PULLUP);
  pinMode(53, OUTPUT); // Declarar pin 53 como salida
  digitalWrite(53, HIGH); // Poner pin 53 en alto => entrega ~5V

  // Calibrar OFFSET del ACS712 en DC
  myACS.autoMidPointDC(1000); // 50 lecturas => Ajusta a tu gusto

  // Definir el ruido
  myACS.setNoisemV(50.88);
}

void loop() {
  // Verificar si hay datos disponibles en el Serial
  while (Serial.available() > 0) {
    char receivedChar = Serial.read(); // Leer el carácter entrante

    // Verificar delimitadores
    if (receivedChar == '<') {
      inputCommand = ""; // Iniciar un nuevo comando
    } else if (receivedChar == '>') {
      processCommand(inputCommand); // Procesar comando completo
      inputCommand = ""; // Limpiar la variable del comando
      lastCommandTime = millis(); // Actualizar el tiempo del último comando recibido
    } else {
      inputCommand += receivedChar; // Agregar carácter al comando actual
    }
  }

  // Actualizar los motores si se está usando PID
  if (usarPID) {
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
  }
}

void processCommand(String command) {
switch (command[0]) {
  case 'm': {
    // Verificar si el segundo carácter es un espacio
    if (command.length() < 3 || command[1] != ' ') {
      Serial.println("<Error: Formato inválido. Debe ser <m valor1 valor2>>");
      break;
    }

    // Eliminar el prefijo "m " (incluyendo el espacio)
    command.remove(0, 2);

    // Verificar si hay exactamente un espacio separando los valores
    int spaceIndex = command.indexOf(' ');
    if (spaceIndex == -1 || spaceIndex == 0 || spaceIndex == command.length() - 1) {
      Serial.println("<Error: Formato de comando inválido>");
      break;
    }

    // Separar las velocidades y convertir a float
    float velMotor1RPS = command.substring(0, spaceIndex).toFloat();
    float velMotor2RPS = command.substring(spaceIndex + 1).toFloat();

    // Configurar las velocidades de los motores
    motor1.setReferenciaVelocidadRPS(velMotor1RPS);
    motor2.setReferenciaVelocidadRPS(velMotor2RPS);
    usarPID = true;

    // Caso especial: detener motores si ambas velocidades son 0
    if (velMotor1RPS == 0 && velMotor2RPS == 0) {
      motor1.controlarMotor(0);
      motor2.controlarMotor(0);
      usarPID = false;
      motor1.desactivarMotor();
      motor2.desactivarMotor();
    }
    break;
  }

    case 'o': {
      // Controlar motores directamente con PWM (sin PID)
      command.remove(0, 1); // Eliminar el prefijo "o"
      int spaceIndex = command.indexOf(' ');
      int pwmMotor1Value = command.substring(0, spaceIndex).toInt();
      int pwmMotor2Value = command.substring(spaceIndex + 1).toInt();

      motor1.controlarMotor(pwmMotor1Value);
      motor2.controlarMotor(pwmMotor2Value);
      usarPID = false;

      Serial.println("<Control PWM directo activado>");
      Serial.print("<Motor 1 (PWM): ");
      Serial.print(pwmMotor1Value);
      Serial.println(">");
      Serial.print("<Motor 2 (PWM): ");
      Serial.print(pwmMotor2Value);
      Serial.println(">");
      break;
    }

    case 'b': {
      Serial.print("<Baudrate actual: ");
      Serial.print(115200);
      Serial.println(">");
      break;
    }

    case 'e': {
      Serial.print("<");
      Serial.print(motor1.leerEncoder());
      Serial.print(",");
      Serial.print(motor2.leerEncoder());
      Serial.println(">");
      break;
    }

    case 'r': {
      motor1.resetEncodersValues();
      motor2.resetEncodersValues();
      Serial.println("<Encoders reseteados>");
      break;
    }

    case 'i': {
      Serial.println("<OK>");
      break;
    }

    case 'v': {
      Serial.print("<");
      Serial.print(motor1.getVelocidadRPS());
      Serial.print(",");
      Serial.print(motor2.getVelocidadRPS());
      Serial.println(">");
      break;
    }

    case 'c': {
      float current_mA = myACS.mA_DC(60); // Leer corriente
      float umbral = 50.0;

      if (current_mA > umbral) {
        // Corriente positiva
      } else if (current_mA < -umbral) {
        // Corriente negativa
      } else {
        current_mA = 0.0;
      }

      Serial.print("<Corriente (mA): ");
      Serial.print(current_mA / 1000);
      Serial.println(">");
      break;
    }

    default: {
      Serial.println("<Comando inválido>");
      break;
    }
  }
}

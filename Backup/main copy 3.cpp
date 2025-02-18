#include <MotorControlPIDV1_Arduino.cpp>

// Instanciar motores
Motor motor1(5, 7, 8, 19, 18, 0.1, 0, 0.015, 30);
Motor motor2(44, A13, A12, 3, 2, 0.1, 0.0, 0.015, 30);

const byte numChars = 100;  // Tamaño del buffer de comandos
char inputCommand[numChars];  // Variable para almacenar el comando recibido
boolean newData = false;      // Flag para indicar que se ha recibido un comando completo
bool usarPID = true;          // Variable para controlar si se usa PID o no
long int Baudrate = 115200;

unsigned long lastCommandTime = 0;   // Tiempo del último comando recibido
const unsigned long timeout = 3000;  // Tiempo de espera para desactivar motores

void leerEncoders();
void detenerMotores();
void recvWithStartEndMarkers(); 
void processCommand(char* command);

void setup() {
  Serial.begin(Baudrate);
  while (!Serial) { /* Esperar a que el puerto serial esté listo */ }

  // Inicializar motores
  motor1.inicializar();
  motor2.inicializar();

  // Mensaje inicial
  Serial.println("<OK>");
}

void loop() {
  recvWithStartEndMarkers();  // Llamar a la función de recepción con delimitadores
  if (newData) {
    processCommand(inputCommand);  // Procesar el comando completo
    newData = false;
    lastCommandTime = millis();    // Actualizar el tiempo del último comando recibido
  }

  // Actualizar los motores solo si se está usando PID
  if (usarPID) {
    motor1.actualizar();
    motor2.actualizar();
  }

  // Verificar si ha pasado el tiempo de espera sin recibir comandos
  if (millis() - lastCommandTime > timeout) {
    if (usarPID) {
      //Serial.println("Timeout alcanzado, desactivando motores.");
    }
    detenerMotores();  // Llamada a la función para detener y desactivar motores
  }
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        inputCommand[ndx] = rc;
        ndx++;
        if (ndx >= numChars) ndx = numChars - 1;  // Prevenir desbordamiento
      } else {
        inputCommand[ndx] = '\0';  // Terminar el comando con carácter nulo
        recvInProgress = false;
        ndx = 0;
        newData = true;            // Indicar que hay un nuevo comando completo
      }
    } else if (rc == startMarker) {
      recvInProgress = true;       // Iniciar la recepción cuando se detecta el marcador de inicio
    }
  }
}

void detenerMotores() {
  motor1.controlarMotor(0);
  motor2.controlarMotor(0);
  motor1.desactivarMotor();
  motor2.desactivarMotor();
  usarPID = false;
}

void processCommand(char* command) {
  switch (command[0]) {
    case 'm': {
      // Comando para configurar velocidades de los motores usando PID
      char* velocidadMotor1 = strtok(command + 2, " ");
      char* velocidadMotor2 = strtok(NULL, " ");

      if (velocidadMotor1 != NULL && velocidadMotor2 != NULL) {
        float velMotor1RPS = atof(velocidadMotor1);
        float velMotor2RPS = atof(velocidadMotor2);

        motor1.setReferenciaVelocidadRPS(velMotor1RPS);
        motor2.setReferenciaVelocidadRPS(velMotor2RPS);

        usarPID = true;
        if (velMotor1RPS == 0 && velMotor2RPS == 0) {
          detenerMotores();
        }
      }
      break;
    }
    case 'o': {
      // Comando para configurar PWM de los motores directamente (sin PID)
      char* pwmMotor1 = strtok(command + 2, " ");
      char* pwmMotor2 = strtok(NULL, " ");

      if (pwmMotor1 != NULL && pwmMotor2 != NULL) {
        int pwmMotor1Value = atoi(pwmMotor1);
        int pwmMotor2Value = atoi(pwmMotor2);

        motor1.controlarMotor(pwmMotor1Value);
        motor2.controlarMotor(pwmMotor2Value);

        usarPID = false;
        Serial.println("Control PWM directo activado:");
        Serial.print("Motor 1 (PWM): ");
        Serial.println(pwmMotor1Value);
        Serial.print("Motor 2 (PWM): ");
        Serial.println(pwmMotor2Value);
      }
      break;
    }
    case 'b': {
      // Comando para devolver el baudrate
      Serial.print("Baudrate actual: ");
      Serial.println(Baudrate);
      break;
    }
    case 'e': {
      // Comando para devolver los valores de los encoders
      leerEncoders();
      break;
    }
    case 'r': {
      // Comando para resetear los encoders
      motor1.inicializar();
      motor2.inicializar();
      break;
    }
    case 'i': {
    Serial.print("OK");  // O usa "OK\r\n" para ser más explícito
    break;
    }
    default: {
      Serial.println("Comando inválido.");
      break;
    }
  }
}

void leerEncoders() {
  // Leer los valores de los encoders de ambos motores y enviar la respuesta
  long encoder1 = motor1.leerEncoder();
  long encoder2 = motor2.leerEncoder();
  String response = String(encoder1) + "," + String(encoder2);
  Serial.println(response);
}

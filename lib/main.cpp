#include <MotorControlPIDV1_Arduino.cpp>
// Instanciar motores
//Motor(int enable, int in1, int in2, int encoderA, int encoderB, float kp, float ki, float kd, unsigned long muestreo) 
Motor motor1(5, 8, 7, 19, 18, 0.1, 0.15, 0.08, 30);
Motor motor2(44, A12, A13, 3, 2, 0.1, 0.15, 0.08, 30);

char inputCommand[100];  // Variable para almacenar el comando recibido
int commandIndex = 0;
void processCommand(char* command);
bool usarPID = true;  // Variable para controlar si se usa PID o no
long int Baudrate = 57600;

void setup() {
  Serial.begin(57600);
  while (!Serial) { /* Esperar a que el puerto serial esté listo */ }

  // Inicializar motores
  motor1.inicializar();
  motor2.inicializar();

  // Imprimir mensaje inicial
  Serial.println("Sistema de comandos iniciado. Ingrese comandos.");
}

void loop() {
  // Verificar si hay datos disponibles en el Serial
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();  // Leer el carácter entrante
    if (receivedChar != '\n') {
      inputCommand[commandIndex++] = receivedChar;  // Agregar carácter al comando actual
      if (commandIndex >= 99) commandIndex = 99;  // Prevenir desbordamiento del buffer
    } else {
      inputCommand[commandIndex] = '\0';  // Terminar el comando con un caracter nulo
      processCommand(inputCommand);  // Procesar el comando
      commandIndex = 0;  // Reiniciar el índice del comando
    }
  }

  // Actualizar los motores en el loop principal, solo si se está usando PID
  if (usarPID) {
    motor1.actualizar();
    motor2.actualizar();
  }
}

void processCommand(char* command) {
  switch (command[0]) {
    case 'm': {
      // Comando para configurar velocidades de los motores usando PID
      char* velocidadMotor1 = strtok(command + 2, " ");
      char* velocidadMotor2 = strtok(NULL, " ");

      if (velocidadMotor1 != NULL && velocidadMotor2 != NULL) {
        // Convertir a float
        float velMotor1RPS = atof(velocidadMotor1);
        float velMotor2RPS = atof(velocidadMotor2);

        // Configurar las velocidades de los motores usando PID
        motor1.setReferenciaVelocidadRPS(velMotor1RPS);
        motor2.setReferenciaVelocidadRPS(velMotor2RPS);

        usarPID = true;  // Asegurarse de que el PID esté activado
        Serial.println("Velocidades establecidas (PID activado):");
        Serial.print("Motor 1 (RPS): ");
        Serial.println(velMotor1RPS);
        Serial.print("Motor 2 (RPS): ");
        Serial.println(velMotor2RPS);

        // Si se envía m 0 0, detener los motores
        if (velMotor1RPS == 0 && velMotor2RPS == 0) {
          motor1.controlarMotor(0);
          motor2.controlarMotor(0);
          usarPID = false;
          Serial.println("Motores detenidos.");
        }
      } else {
        Serial.println("Error: Comando de velocidad incompleto.");
      }
      break;
    }
    case 'o': {
      // Comando para configurar PWM de los motores directamente (sin PID)
      char* pwmMotor1 = strtok(command + 2, " ");
      char* pwmMotor2 = strtok(NULL, " ");

      if (pwmMotor1 != NULL && pwmMotor2 != NULL) {
        // Convertir a entero
        int pwmMotor1Value = atoi(pwmMotor1);
        int pwmMotor2Value = atoi(pwmMotor2);

        // Controlar los motores directamente con PWM (sin PID)
        motor1.controlarMotor(pwmMotor1Value);
        motor2.controlarMotor(pwmMotor2Value);

        usarPID = false;  // Desactivar el uso de PID
        Serial.println("Control PWM directo activado:");
        Serial.print("Motor 1 (PWM): ");
        Serial.println(pwmMotor1Value);
        Serial.print("Motor 2 (PWM): ");
        Serial.println(pwmMotor2Value);
      } else {
        Serial.println("Error: Comando de PWM incompleto.");
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
      Serial.print("Encoder motor 1: ");
      Serial.println(motor1.leerEncoder());
      Serial.print("Encoder motor 2: ");
      Serial.println(motor2.leerEncoder());
      break;
    }
    case 'r': {
      // Comando para resetear los encoders
      motor1.inicializar();  // Esto reinicia el encoder del motor 1
      motor2.inicializar();  // Esto reinicia el encoder del motor 2
      Serial.println("Encoders reiniciados.");
      break;
    }
    default: {
      // Comando inválido
      Serial.println("Comando inválido.");
      break;
    }
  }
}

#include <MotorControlPIDV1_Arduino.cpp>
#include <ACS712.h>


// === CONFIGURACIÓN DEL SENSOR ACS712 ===
// Usar el modelo de 20A como ejemplo. Cambia a 05B o 30A si corresponde.
ACS712  myACS(A15, 5.0, 1023, 185);


// Instanciar motores
Motor motor1(5, 7, 8, 19, 18, 0.1, 0, 0.015, 1);
Motor motor2(44, A13, A12, 3, 2, 0.1, 0.0, 0.015, 1);

String inputCommand = "";  // Variable para almacenar el comando recibido
void processCommand(String command);
bool usarPID = true;  // Variable para controlar si se usa PID o no
unsigned long lastCommandTime = 0;  // Variable para almacenar el tiempo del último comando recibido
const unsigned long timeout = 3000; // Tiempo de espera (3 segundos)

void setup() {
  Serial.begin(115200);
  Serial.println("Inicializando el Arduino Mega (Rebooting)");
  
  // Inicializar motores
  motor1.inicializar();
  motor2.inicializar();

  //Inicializamos el VCC del ACS712
  pinMode(53, INPUT_PULLUP);
  pinMode(53, OUTPUT);     // Declarar pin 53 como salida
  digitalWrite(53, HIGH);  // Poner pin 53 en alto => entrega ~5V
  // Mensaje inicial (opcional)
  // Serial.println("Sistema de comandos iniciado. Ingrese comandos.");

  // === 3) CALIBRAR OFFSET DEL ACS712 EN DC  ===
  // Usa N lecturas para encontrar el midPoint cuando NO hay corriente.
  // En lo posible, que el motor esté inactivo o con muy poca carga.
  myACS.autoMidPointDC(1000);  // 50 lecturas => Ajusta a tu gusto

  //Definimos el ruido.
  myACS.setNoisemV(50.88);

} 

void loop() {
  // Verificar si hay datos disponibles en el Serial
  if (Serial.available() > 0) {
    char receivedChar = Serial.read();  // Leer el carácter entrante
    inputCommand += receivedChar;       // Agregar carácter al comando actual

    // Procesar el comando al recibir un salto de línea (fin de comando)
    if (receivedChar == '\n') {
      inputCommand.trim();  // Eliminar espacios en blanco y caracteres no visibles
      processCommand(inputCommand);  // Procesar el comando
      inputCommand = "";  // Limpiar la variable del comando para recibir el siguiente
      lastCommandTime = millis();  // Actualizar el tiempo del último comando recibido
    }
  }

  // Actualizar los motores en el loop principal, solo si se está usando PID
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
  if (command.startsWith("m ")) {
    // Comando para configurar velocidades de los motores usando PID
    command.remove(0, 2);  // Eliminar el prefijo "m "
    
    // Dividir los valores de velocidad
    int spaceIndex = command.indexOf(' ');
    String velocidadMotor1 = command.substring(0, spaceIndex);
    String velocidadMotor2 = command.substring(spaceIndex + 1);

    // Convertir a float
    float velMotor1RPS = velocidadMotor1.toFloat();
    float velMotor2RPS = velocidadMotor2.toFloat();

    // Configurar las velocidades de los motores usando PID
    motor1.setReferenciaVelocidadRPS(velMotor1RPS);
    motor2.setReferenciaVelocidadRPS(velMotor2RPS);

    usarPID = true;
    
    // Detener los motores si las velocidades son cero
    if (velMotor1RPS == 0 && velMotor2RPS == 0) {
      motor1.controlarMotor(0);
      motor2.controlarMotor(0);
      usarPID = false;
      motor1.desactivarMotor();
      motor2.desactivarMotor();
    }
  } 
  else if (command.startsWith("o ")) {
    // Comando para configurar PWM de los motores directamente (sin PID)
    command.remove(0, 2);  // Eliminar el prefijo "o "
    
    // Dividir los valores de PWM
    int spaceIndex = command.indexOf(' ');
    String pwmMotor1 = command.substring(0, spaceIndex);
    String pwmMotor2 = command.substring(spaceIndex + 1);

    // Convertir a entero
    int pwmMotor1Value = pwmMotor1.toInt();
    int pwmMotor2Value = pwmMotor2.toInt();

    // Controlar los motores directamente con PWM (sin PID)
    motor1.controlarMotor(pwmMotor1Value);
    motor2.controlarMotor(pwmMotor2Value);

    usarPID = false;
    Serial.println("Control PWM directo activado:");
    Serial.print("Motor 1 (PWM): ");
    Serial.println(pwmMotor1Value);
    Serial.print("Motor 2 (PWM): ");
    Serial.println(pwmMotor2Value);
  } 
  else if (command == "b") {
    // Comando para devolver el baudrate
    Serial.print("Baudrate actual: ");
    Serial.println(115200);  // Reemplaza Serial.baudRate() con el valor configurado
  } 
  else if (command == "e") {
    // Comando para devolver los valores de los encoders
    Serial.print(motor1.leerEncoder());
    Serial.print(",");
    Serial.println(motor2.leerEncoder());
  } 
  else if (command == "r") {
    // Comando para resetear los encoders
    motor1.inicializar();
    motor2.inicializar();
  } 
  else if (command == "i") {
    Serial.print("OK");
  } 
 else if (command == "v") {
    // Comando para devolver la velocidad actual de los motores en RPS
    Serial.print(motor1.getVelocidadRPS());
    Serial.print(",");
    Serial.println(motor2.getVelocidadRPS());
}else if (command == "c") {
    // == Comando "c" => Leer corriente DC en mA ==
    // Toma 'X' lecturas y promedia, ajusta X según necesites más estabilidad.
    float current_mA =  myACS.mA_DC(60);  //myACS.mVNoiseLevel();//myACS.mA_DC(60);  // 10 lecturas
      float umbral = 50.0;                  // 130 mA de zona muerta

  if (current_mA > umbral) {
  // Corriente positiva > 130 mA
  // => dirección “normal” o forward
    } 
  else if (current_mA < -umbral) {
  // Corriente negativa < -130 mA
  // => dirección inversa o reverse
    }
  else {
  // -130 mA <= corriente <= +130 mA
  // => asumir 0
  current_mA = 0.0;}

    Serial.print("Corriente (mA): ");
    Serial.println(current_mA/1000);
  }

  else {
    // Comando inválido
    Serial.println("Comando inválido.");
  }
}

#include <Arduino.h>
#include <Encoder.h>

// Definición de pines
const int pinPWM = 44;
const int pinDir1 = A12;
const int pinDir2 = A13;

const int pinEncoderA = 2;
const int pinEncoderB = 3;

// Variables del PID
float referenciaVelocidad = 12780/4; // Velocidad deseada en RPM o pulsos por segundo
float errorActual = 0;
float errorPrevio = 0;
float sumaErrores = 0;
float derivadaError = 0;
float kp = 0.1;
float ki = 0.0;
float kd = 0.015;
float salidaPID = 0;
unsigned long tiempoPrevio = 0;
unsigned long intervaloMuestreo = 30; // Tiempo de muestreo en milisegundos

// Variable de encoder
Encoder encoder(pinEncoderA, pinEncoderB);
long posicionEncoder = 0;
float velocidadActual = 0;

long EncoderValueForDebug = 0;
unsigned long TimeValueForDebugging = 0;
float RPSVelocityValueForDebug = 0;

// Prototipos de funciones
float calcularPID(float referencia, float actual);
void controlarMotor(float valorPID);
long leerEncoder();
float calcularVelocidad(long posicionActual, long posicionAnterior, unsigned long tiempoAnterior);
float convertirTicksARPS(long ticksPorSegundo);

void setup() {
    Serial.begin(115200);

    // Configuración de los pines de motor
    pinMode(pinPWM, OUTPUT);
    pinMode(pinDir1, OUTPUT);
    pinMode(pinDir2, OUTPUT);

    // Tiempo inicial
    tiempoPrevio = millis();
}

void loop() {
    // Controlar la velocidad de muestreo
    unsigned long tiempoActual = millis();
    if (tiempoActual - tiempoPrevio >= intervaloMuestreo) {

        TimeValueForDebugging = tiempoActual - tiempoPrevio;
        // Leer la posición del encoder
        long posicionAnterior = posicionEncoder;
        posicionEncoder = leerEncoder();

        // Calcular la velocidad actual del motor
        velocidadActual = calcularVelocidad(posicionEncoder, posicionAnterior, tiempoPrevio);
        tiempoPrevio = tiempoActual; // Actualizar el tiempo previo

        // Calcular el valor de control PID
        salidaPID = calcularPID(referenciaVelocidad, velocidadActual);

        // Controlar el motor con el valor calculado del PID
        controlarMotor(salidaPID);
        EncoderValueForDebug = posicionEncoder - posicionAnterior;

        RPSVelocityValueForDebug = convertirTicksARPS(velocidadActual);
        // Monitoreo en el serial
        Serial.print("Velocidad Actual (RPS): ");
        Serial.print(RPSVelocityValueForDebug);
        Serial.print(" | Salida PID: ");
        Serial.print(salidaPID);
        Serial.print(" Ver Tiempo.: ");
        Serial.print(millis());
        Serial.print(" TimeEncoder.: ");
        Serial.print(TimeValueForDebugging); // Elimine millis()
        Serial.print(" EncoderValue.: ");
        Serial.println(EncoderValueForDebug);
    }
}

// Función para calcular el PID
float calcularPID(float referencia, float actual) {
    errorActual = referencia - actual;
    sumaErrores += errorActual;
    derivadaError = errorActual - errorPrevio;
    
    float salida = (kp * errorActual) + (ki * sumaErrores) + (kd * derivadaError);
    
    errorPrevio = errorActual; // Actualizar el error previo
    
    // Limitar el valor de salida para mantenerlo dentro de los rangos válidos del PWM
    if (salida > 255) salida = 255;
    if (salida < -255) salida = -255;

    return salida;
}

// Función para controlar el motor en base al valor del PID
void controlarMotor(float valorPID) {
    if (valorPID > 0) {
        // Movimiento en sentido horario
        digitalWrite(pinDir1, HIGH);
        digitalWrite(pinDir2, LOW);
        analogWrite(pinPWM, abs(valorPID)); // PWM en el rango de 0 a 255
    } else if (valorPID < 0) {
        // Movimiento en sentido antihorario
        digitalWrite(pinDir1, LOW);
        digitalWrite(pinDir2, HIGH);
        analogWrite(pinPWM, abs(valorPID)); // PWM en el rango de 0 a 255
    } else {
        // Motor detenido
        analogWrite(pinPWM, 0);
    }
}

// Función para leer el encoder
long leerEncoder() {
    return encoder.read();
}

// Función para calcular la velocidad (pulsos por segundo o RPM)
float calcularVelocidad(long posicionActual, long posicionAnterior, unsigned long tiempoAnterior) {
    long deltaPosicion = posicionActual - posicionAnterior;
    unsigned long deltaTiempo = millis() - tiempoAnterior;
    float velocidad = (deltaPosicion / (float)deltaTiempo) * 1000; // Pulsos por segundo

    // Convertir si es necesario a RPM o la unidad que estés utilizando
    return velocidad;
}

float convertirTicksARPS(long ticksPorSegundo) {
    const float pulsosPorRevolucion = 270.0 * 64; // Pulsos del encoder por cada revolución completa

    // Convertir los ticks por segundo a revoluciones por segundo (RPS)
    float rps = (float)ticksPorSegundo / pulsosPorRevolucion;

    return rps ; // Me da que 1 Revolución, es 2 segundos.
}

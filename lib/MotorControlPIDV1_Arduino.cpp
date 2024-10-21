#include <Arduino.h>
#include <Encoder.h>

class Motor {
  private:
    int pinEnable;
    int pinIN1;
    int pinIN2;
    int pinEncoderA;
    int pinEncoderB;
    float kp, ki, kd;
    float referenciaVelocidad;
    float errorActual;
    float errorPrevio;
    float sumaErrores;
    float derivadaError;
    unsigned long tiempoPrevio;
    unsigned long intervaloMuestreo;
    Encoder encoder;
    long posicionEncoder;
    float velocidadActual;
    const float pulsosPorRevolucion = 4320.0 * 2; // Pulsos del encoder por revolución
    float valorPWM; // Nueva variable para almacenar el valor actual del PWM
    float ajuste = 1;
  public:
    Motor(int enable, int in1, int in2, int encoderA, int encoderB, float kp, float ki, float kd, unsigned long muestreo) 
      : pinEnable(enable), pinIN1(in1), pinIN2(in2), pinEncoderA(encoderA), pinEncoderB(encoderB), kp(kp), ki(ki), kd(kd), 
        referenciaVelocidad(0), errorActual(0), errorPrevio(0), sumaErrores(0), derivadaError(0), intervaloMuestreo(muestreo), 
        encoder(pinEncoderA, pinEncoderB), posicionEncoder(0), velocidadActual(0), valorPWM(0) {
    }

    void inicializar() {
      // Configuración de los pines de motor
      pinMode(pinEnable, OUTPUT);
      pinMode(pinIN1, OUTPUT);
      pinMode(pinIN2, OUTPUT);

      // Configuración del encoder
      posicionEncoder = encoder.read();

      // Inicializar tiempo
      tiempoPrevio = millis();
    }

    // Configuración de velocidad por ticks por segundo
    void setReferenciaVelocidad(float referencia) {
      referenciaVelocidad = referencia;
    }

    // Configuración de velocidad por RPS (Revoluciones por segundo)
    void setReferenciaVelocidadRPS(float rps) {
      referenciaVelocidad = (rps * pulsosPorRevolucion) / ajuste; // Convertir RPS a ticks por segundo
    }

    // Configuración de velocidad por RPM (Revoluciones por minuto)
    void setReferenciaVelocidadRPM(float rpm) {
      float rps = rpm / 60.0;  // Convertir RPM a RPS
      referenciaVelocidad = (rps * pulsosPorRevolucion) / ajuste; // Convertir RPS a ticks por segundo
    }

    void actualizar() {
      // Controlar la velocidad de muestreo
      unsigned long tiempoActual = millis();
      if (tiempoActual - tiempoPrevio >= intervaloMuestreo) {
        long posicionAnterior = posicionEncoder;
        posicionEncoder = leerEncoder();

        velocidadActual = calcularVelocidad(posicionEncoder, posicionAnterior, tiempoPrevio);
        tiempoPrevio = tiempoActual;

        valorPWM = calcularPID(referenciaVelocidad, velocidadActual); // Guardar el valor del PWM
        controlarMotor(valorPWM);
      }
    }

    long leerEncoder() {
      return encoder.read();
    }

    float calcularVelocidad(long posicionActual, long posicionAnterior, unsigned long tiempoAnterior) {
      long deltaPosicion = posicionActual - posicionAnterior;
      unsigned long deltaTiempo = millis() - tiempoAnterior;
      float velocidad = (deltaPosicion / (float)deltaTiempo) * 1000;
      return velocidad;
    }

    float calcularPID(float referencia, float actual) {
      errorActual = referencia - actual;
      sumaErrores += errorActual;
      derivadaError = errorActual - errorPrevio;

      float salida = (kp * errorActual) + (ki * sumaErrores) + (kd * derivadaError);
      errorPrevio = errorActual;

      if (salida > 255) salida = 255;
      if (salida < -255) salida = -255;

      return salida;
    }
    void controlarMotor(float valorPID) {
      if (valorPID > 0) {
        digitalWrite(pinIN1, HIGH);
        digitalWrite(pinIN2, LOW);
        analogWrite(pinEnable, (int)abs(valorPID));
      } else if (valorPID < 0) {
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, HIGH);
        analogWrite(pinEnable, (int)abs(valorPID));
      } else {
        analogWrite(pinEnable, 0);
      }
    }

    // Obtener la velocidad en ticks por segundo
    float getVelocidadTicksPorSegundo() {
      return velocidadActual;
    }

    // Obtener la velocidad en RPS (Revoluciones por segundo)
    float getVelocidadRPS() {
      return (velocidadActual / pulsosPorRevolucion);
    }

    // Obtener la velocidad en RPM (Revoluciones por minuto)
    float getVelocidadRPM() {
      return ((velocidadActual / pulsosPorRevolucion) * 60.0);
    }

    // Obtener el valor actual del PWM
    float getValorPWM() {
      return valorPWM;
    }
};

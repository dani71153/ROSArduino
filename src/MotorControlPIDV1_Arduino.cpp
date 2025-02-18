#include <Arduino.h>
#include <Encoder.h>

/**
 * Características y Funcionalidades del Código (Antes de los Cambios)
 *
 * 1. Configuración del Motor y Encoder:
 *    - Configura los pines necesarios para controlar el motor (pines de dirección, habilitación y encoder).
 *    - Utiliza la librería ESP32Encoder para manejar la lectura del encoder en modo cuadratura.
 *
 * 2. Control PID:
 *    - Implementa un controlador PID para regular la velocidad del motor en función de una referencia.
 *    - Calcula los términos proporcional, integral y derivativo para ajustar el valor PWM que controla la velocidad del motor.
 *    - Incluye saturación del valor de salida del PID al rango permitido por el PWM (de -255 a 255).
 *
 * 3. Control del Motor:
 *    - Controla el sentido de giro del motor según el valor del PID.
 *    - Genera señales PWM utilizando un canal configurado para el ESP32 con la función ledcWrite.
 *
 * 4. Velocidades en Diferentes Unidades:
 *    - Permite establecer la velocidad de referencia en ticks por segundo, RPS (revoluciones por segundo) o RPM (revoluciones por minuto).
 *    - Proporciona métodos para obtener la velocidad actual en estas mismas unidades.
 *
 * 5. Frecuencia y Resolución del PWM:
 *    - Configura la frecuencia y resolución del PWM mediante métodos dedicados.
 *    - Es posible cambiar estos parámetros durante la ejecución.
 *
 * 6. Lectura del Encoder y Cálculo de Velocidad:
 *    - Calcula la velocidad actual del motor en función de los pulsos del encoder y el tiempo transcurrido.
 *    - La velocidad se mide en ticks por segundo.
 *
 * 7. Actualización periódica:
 *    - El método `actualizar` verifica si ha transcurrido el intervalo de muestreo para realizar las operaciones de lectura del encoder, cálculo del PID y actualización del motor.
 *
 * Agregados hoy 14 de Enero:
 *
 * 1. Parada Activa:
 *    - Se implementó un mecanismo de frenado activo en el método controlarMotor. Esto asegura que el motor se detenga de manera rápida y precisa, reduciendo el deslizamiento.
 *    - La parada activa consiste en poner ambos pines de dirección del motor en HIGH cuando el valor del PID es 0.
 *
 * 2. Anti-Windup:
 *    - Se añadió una limitación a la acumulación de errores en el cálculo del PID (término integral), para evitar el windup.
 *    - Esto asegura que el término integral no crezca indefinidamente, lo que mejora la respuesta del sistema y evita comportamientos inestables.
 * Para la nota de hoy de la bitacora. Se puede agregar que para mejoras futuras, se puede trabajar en un PID que corrija de mejor manera a velocidades más altas.
 * Tambien, se cambiaron los tiempos de el PID a 1 ms para el muestreo.
 *
 * 
 *  */

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
    const float pulsosPorRevolucion = 270 * 64; // Pulsos del encoder por revolución
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



/** Version Original de Calcular PID, no tiene el decremento y ajuste de las ganancias a medida que aumenta la velocidad.*/
    float calcularPID(float referencia, float actual) {
        if(referencia == 0)
      {
        errorActual=0;
        sumaErrores =0;
      }

      errorActual = referencia - actual;
      sumaErrores += errorActual;
      //Linea Agregada el 23 de Enero. A ver si mejora el antiwindup.

    // Evitamos la acumulación descontrolada del error integral si la salida está saturada
    /*if (valorPWM < 255 && valorPWM > -255) {
        sumaErrores += errorActual;
    }*/
      //Definimos un antiwindup. Para evitar la acumulacion de errores.
      if (sumaErrores > 1000) sumaErrores = 1000; // Ajusta según tus necesidades
      if (sumaErrores < -1000) sumaErrores = -1000;

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
        //Agregamos una parada.
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, HIGH);
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

    // Nueva función para desactivar el motor (desactivar el enable del L298N)
    void desactivarMotor() {
      analogWrite(pinEnable, 0); // Poner el PWM a 0
      digitalWrite(pinIN1, LOW); // Apagar las entradas del motor
      digitalWrite(pinIN2, LOW); // Apagar las entradas del motor
    }


  void resetEncodersValues(){
  encoder.write(0);
}

};
/* Modificaciones realizadas:
1. Se agregó la función `desactivarMotor()` para desactivar el enable del controlador L298N y poner a LOW las entradas del motor.
   - Esta función se utiliza para desactivar completamente el motor cuando no se reciben comandos.
*/

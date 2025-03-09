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
    float errorActual, errorPrevio, sumaErrores, derivadaError;
    unsigned long tiempoPrevio;
    unsigned long intervaloMuestreo;
    Encoder encoder;
    long posicionEncoder;
    float velocidadActual;
    const float pulsosPorRevolucion = 270 * 64; // Pulsos del encoder por revolución
    float valorPWM; // Nueva variable para almacenar el valor actual del PWM
    float ajuste = 1;
    float referenciaAnterior = 0.0; // Nueva variable para almacenar la referencia anterior
    float valorPIDAnterior = 0.0; // Valor anterior del PID
    float maxCambioRampa = 2;  // Cambio máximo permitido por iteración
    static const int numLecturasFiltro = 5; // Número de lecturas para el filtro de media móvil
    long bufferLecturas[numLecturasFiltro]; // Buffer para almacenar las lecturas del encoder
    int indiceFiltro; // Índice para las lecturas del filtro
    long sumaLecturas; // Suma de las lecturas para el filtro
    float salidaIIR; // Variable para almacenar la salida del filtro IIR
    float alpha = 0.5; // Coeficiente de suavizado para el filtro IIR

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

      // Inicializar tiempo y lecturas del encoder
      tiempoPrevio = millis();
      indiceFiltro = 0;
      sumaLecturas = 0;
      memset(bufferLecturas, 0, sizeof(bufferLecturas));
      salidaIIR = 0; // Inicializar la salida del filtro IIR
    }

    // Configuración de velocidad por ticks por segundo
    void setReferenciaVelocidad(float referencia) {
        // Verificar si hay un cambio de referencia
        if (referencia != referenciaAnterior && referencia != 0) {
            errorActual = 0; // Resetear el error actual
            sumaErrores = 0; // Resetear la suma de errores
        }
        referenciaAnterior = referencia; // Actualizar la referencia anterior
        referenciaVelocidad = referencia; // Asignar la nueva referencia
    }

    // Configuración de velocidad por RPS (Revoluciones por segundo)
    void setReferenciaVelocidadRPS(float rps) {
      float nuevaReferencia = (rps * pulsosPorRevolucion) / ajuste; // Convertir RPS a ticks por segundo

      // Verificar si hay un cambio de referencia
      if (nuevaReferencia != referenciaAnterior && nuevaReferencia != 0) {
          errorActual = 0; // Resetear el error actual
          sumaErrores = 0; // Resetear la suma de errores
      }
      referenciaAnterior = nuevaReferencia; // Actualizar la referencia anterior
      referenciaVelocidad = nuevaReferencia; // Asignar la nueva referencia
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
      }
      controlarMotor(valorPWM);

    }

    long leerEncoder() {
      // Leer el valor actual del encoder
      long lecturaActual = encoder.read();

      // Actualizar el buffer y la suma para el filtro de media móvil
      sumaLecturas -= bufferLecturas[indiceFiltro];
      bufferLecturas[indiceFiltro] = lecturaActual;
      sumaLecturas += lecturaActual;

      // Avanzar el índice del buffer
      indiceFiltro = (indiceFiltro + 1) % numLecturasFiltro;

      // Salida del filtro de media móvil
      long salidaMediaMovil = sumaLecturas / numLecturasFiltro;

      // Aplicar el filtro IIR en cascada
      salidaIIR = alpha * salidaMediaMovil + (1 - alpha) * salidaIIR;

      // Retornar el valor filtrado por el IIR
      return salidaIIR;
    }

    float calcularVelocidad(long posicionActual, long posicionAnterior, unsigned long tiempoAnterior) {
      // Usar el valor filtrado del encoder
      long deltaPosicion = posicionActual - posicionAnterior;
      unsigned long deltaTiempo = millis() - tiempoAnterior;

      // Evitar división por cero
      if (deltaTiempo == 0) {
          return 0;
      }

      // Calcular la velocidad en ticks por segundo
      float velocidad = (deltaPosicion / (float)deltaTiempo) * 1000;

      // Aplicar un filtro adicional si es necesario
      // Por ejemplo, un filtro de media móvil o un filtro de Kalman

      return velocidad;
    }

    float calcularPID(float referencia, float actual) {
      errorActual = referencia - actual;
      sumaErrores += errorActual;
      
      // Agregamos una proteccion atraves de la suma de los errores.  Para ponerle un limite.
      if (sumaErrores > 65536) sumaErrores = 65536;
      if (sumaErrores < -65536) sumaErrores = -65536;

      derivadaError = errorActual - errorPrevio;

      float salidaSinLimitar = (kp * errorActual) + (ki * sumaErrores) + (kd * derivadaError);
      errorPrevio = errorActual;

      // Saturación normal
      if (salidaSinLimitar > 255) salidaSinLimitar = 255;
      if (salidaSinLimitar < -255) salidaSinLimitar = -255;
      
      // Aplicar limitador de rampa
      float cambio = salidaSinLimitar - valorPIDAnterior;
      
      // Limitar la tasa de cambio
      if (cambio > maxCambioRampa)
        cambio = maxCambioRampa;
      else if (cambio < -maxCambioRampa)
        cambio = -maxCambioRampa;
      
      float salidaLimitada = valorPIDAnterior + cambio;
      valorPIDAnterior = salidaLimitada;
      
      return salidaLimitada;
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
        // Modificar el comportamiento del freno activo
        // Aplicar freno solo si el motor debe detenerse completamente
        if (referenciaVelocidad == 0) {
            digitalWrite(pinIN1, HIGH);
            digitalWrite(pinIN2, HIGH); // Freno activo
            analogWrite(pinEnable, 0);
        } else {
            // Mantener el último estado del motor
            analogWrite(pinEnable, 0);
        }
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

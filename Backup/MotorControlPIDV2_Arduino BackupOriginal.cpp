#include <Arduino.h>
#include <Encoder.h>
#include "RampaVelocidad.cpp"


/**
 * @class Motor
 * @brief Librería para el control de un motor DC con encoder utilizando un driver BTS7960.
 *
 * @details
 * Esta clase encapsula la funcionalidad completa para controlar un motor DC.
 * Implementa un controlador PID para regular la velocidad del motor basándose en la
 * retroalimentación de un encoder de cuadratura. La librería está diseñada para ser
 * utilizada con el driver de alta potencia BTS7960 (o IBT-2).
 *
 * ## Características Principales:
 *
 * 1.  **Compatibilidad con BTS7960:**
 *     - Utiliza los pines RPWM, LPWM, R_EN y L_EN para un control preciso.
 *     - **Comportamiento Corregido:** Activa los pines de habilitación (R_EN, L_EN)
 *       cada vez que se envía un comando de movimiento para asegurar que el driver
 *       responda correctamente, evitando estados de protección.
 *
 * 2.  **Control PID Integrado:**
 *     - Regula la velocidad del motor para que coincida con un valor de referencia (setpoint).
 *     - Incluye constantes sintonizables (Kp, Ki, Kd).
 *     - Implementa un mecanismo Anti-Windup para el término integral.
 *
 * 3.  **Manejo de Velocidad Flexible:**
 *     - Permite establecer y leer la velocidad en Ticks/s, RPS y RPM.
 *
 * 4.  **Lectura de Encoder:**
 *     - Utiliza la librería "Encoder" de Paul Stoffregen, optimizada para Arduino.
 *
 * 5.  **Control de Motor Avanzado:**
 *     - **Frenado Activo:** Al poner la velocidad en 0, ambos PWM se ponen en bajo.
 *     - **Desactivación (Coast/Freewheel):** El método `desactivarMotor()` apaga
 *       completamente el driver para que el motor gire libremente.
 *
 * @note El valor de `pulsosPorRevolucion` debe ser ajustado según las especificaciones
 *       del motor y encoder que se estén utilizando. Es el producto de los pulsos por
 *       vuelta del encoder y la relación de la caja reductora.
 */
class Motor {
private:
    // --- Pines de control del Motor (BTS7960) ---
    int pinRPWM;        // Pin PWM para la rotación hacia adelante (Right PWM)
    int pinLPWM;        // Pin PWM para la rotación hacia atrás (Left PWM)
    int pinR_EN;        // Pin de habilitación para la rotación hacia adelante (Right Enable)
    int pinL_EN;        // Pin de habilitación para la rotación hacia atrás (Left Enable)

    // --- Pines del Encoder ---
    int pinEncoderA;
    int pinEncoderB;

    // --- Constantes y Variables del PID ---
    float kp, ki, kd;
    float referenciaVelocidad; // Setpoint en ticks/segundo
    float errorActual;
    float errorPrevio;
    float sumaErrores;
    float derivadaError;

    // --- Temporización y Muestreo ---
    unsigned long tiempoPrevio;
    unsigned long intervaloMuestreo;

    // --- Encoder y Estado del Motor ---
    Encoder encoder;
    long posicionEncoder;
    float velocidadActual; // Velocidad medida en ticks/segundo
    float valorPWM;        // Valor de salida del PID, usado para el control

    // --- Constantes del Motor/Encoder ---
    const float pulsosPorRevolucion = 270 * 64;
    float currentReferenciaVelocidad = 0;  // velocidad real usada en PID
    float maxAceleracion = 10000;          // ticks/s², ajústalo
    RampaVelocidad rampa;


public:
    /**
     * @brief Constructor de la clase Motor para el driver BTS7960.
     * @param rpwm Pin de control PWM para la rotación hacia adelante.
     * @param lpwm Pin de control PWM para la rotación hacia atrás.
     * @param r_en Pin de habilitación del lado derecho del puente H.
     * @param l_en Pin de habilitación del lado izquierdo del puente H.
     * @param encoderA Pin A del encoder de cuadratura.
     * @param encoderB Pin B del encoder de cuadratura.
     * @param Kp Ganancia Proporcional del controlador PID.
     * @param Ki Ganancia Integral del controlador PID.
     * @param Kd Ganancia Derivativa del controlador PID.
     * @param muestreo Intervalo de tiempo en milisegundos para actualizar el PID.
     */
    Motor(int rpwm, int lpwm, int r_en, int l_en, int encoderA, int encoderB, float Kp, float Ki, float Kd, unsigned long muestreo)
        : pinRPWM(rpwm), pinLPWM(lpwm), pinR_EN(r_en), pinL_EN(l_en),
          pinEncoderA(encoderA), pinEncoderB(encoderB),
          kp(Kp), ki(Ki), kd(Kd),
          intervaloMuestreo(muestreo),
          encoder(encoderA, encoderB),
          rampa(45000.0f) // o el valor de aceleración máxima que desees //Valores interesantes == 50000

    {
        // Inicializar variables de estado
        referenciaVelocidad = 0;
        errorActual = 0;
        errorPrevio = 0;
        sumaErrores = 0;
        derivadaError = 0;
        posicionEncoder = 0;
        velocidadActual = 0;
        valorPWM = 0;
    }

    /**
     * @brief Configura los pines del motor y encoder. El driver se mantiene desactivado.
     * @details Debe ser llamado en la función `setup()` de Arduino.
     */
    void inicializar() {
        // Configurar pines del motor como salidas
        pinMode(pinRPWM, OUTPUT);
        pinMode(pinLPWM, OUTPUT);
        pinMode(pinR_EN, OUTPUT);
        pinMode(pinL_EN, OUTPUT);

        // Asegurarse de que el driver y los motores estén completamente apagados al inicio.
        digitalWrite(pinR_EN, LOW); // Iniciar con el driver DESACTIVADO
        digitalWrite(pinL_EN, LOW); // Iniciar con el driver DESACTIVADO
        analogWrite(pinRPWM, 0);
        analogWrite(pinLPWM, 0);
        
        posicionEncoder = encoder.read();
        tiempoPrevio = millis();
    }

    /** @brief Establece la velocidad de referencia en ticks por segundo. */
    void setReferenciaVelocidad(float referencia) {
        referenciaVelocidad = referencia;
    }

    /** @brief Establece la velocidad de referencia en Revoluciones Por Segundo (RPS). */
    void setReferenciaVelocidadRPS(float rps) {
        referenciaVelocidad = rps * pulsosPorRevolucion;
    }

    /** @brief Establece la velocidad de referencia en Revoluciones Por Minuto (RPM). */
    void setReferenciaVelocidadRPM(float rpm) {
        float rps = rpm / 60.0;
        referenciaVelocidad = rps * pulsosPorRevolucion;
    }

    /**
     * @brief Método principal que debe ser llamado repetidamente en el `loop()`.
     */
    void actualizar() {
        unsigned long tiempoActual = millis();
        if (tiempoActual - tiempoPrevio >= intervaloMuestreo) {
            long posicionAnterior = posicionEncoder;
            posicionEncoder = leerEncoder();
            velocidadActual = calcularVelocidad(posicionEncoder, posicionAnterior, tiempoPrevio);
            tiempoPrevio = tiempoActual;
            float referenciaSuavizada = rampa.actualizar(referenciaVelocidad);
            valorPWM = calcularPID(referenciaSuavizada, velocidadActual);
            controlarMotor(valorPWM);
        }
    }



    /** @return El número de ticks acumulados leídos por el encoder. */
    long leerEncoder() {
        return encoder.read();
    }

    /** @brief Calcula la velocidad instantánea del motor en ticks por segundo. */
    float calcularVelocidad(long posActual, long posAnterior, unsigned long tiempoAnterior) {
        long deltaPosicion = posActual - posAnterior;
        unsigned long deltaTiempo = millis() - tiempoAnterior;
        if (deltaTiempo == 0) return 0;
        return (deltaPosicion / (float)deltaTiempo) * 1000.0f;
    }

    /** @brief Calcula la salida del controlador PID. */
    float calcularPID(float referencia, float actual) {
        if (referencia == 0) {
            errorActual = 0;
            sumaErrores = 0;
        } else {
            errorActual = referencia - actual;
            sumaErrores += errorActual;
        }

        // Anti-Windup
        if (sumaErrores > 2000) sumaErrores = 2000;
        if (sumaErrores < -2000) sumaErrores = -2000;

        derivadaError = errorActual - errorPrevio;
        errorPrevio = errorActual;
        
        float salida = (kp * errorActual) + (ki * sumaErrores) + (kd * derivadaError);
        return constrain(salida, -255.0, 255.0);
    }

    /**
     * @brief Aplica la señal de control PWM al driver BTS7960, habilitándolo primero.
     * @param valorPID La salida del controlador PID (-255 a 255).
     */
    void controlarMotor(float valorPID) {
        // Habilitar siempre el driver antes de enviar un comando PWM.
        // Esto "despierta" al driver y asegura que responda.
        digitalWrite(pinR_EN, HIGH);
        digitalWrite(pinL_EN, HIGH);

        int pwmValue = (int)abs(valorPID);
        
        // Se añade una pequeña "zona muerta" para evitar movimientos por ruido del PID cerca de cero.
        if (valorPID > 1.0) { 
            // Rotación hacia adelante
            analogWrite(pinLPWM, 0);
            analogWrite(pinRPWM, pwmValue);
        } else if (valorPID < -1.0) {
            // Rotación hacia atrás
            analogWrite(pinRPWM, 0);
            analogWrite(pinLPWM, pwmValue);
        } else {
            // Frenado activo (Brake)
            analogWrite(pinRPWM, 0);
            analogWrite(pinLPWM, 0);
        }
    }

    /**
     * @brief Desactiva completamente el driver del motor (modo "Coast" o "Freewheel").
     * @details Pone los pines de habilitación (R_EN, L_EN) en bajo. El motor girará
     *          libremente. Útil para ahorrar energía o mover el robot manualmente.
     */
    void desactivarMotor() {
        // Poner los pines PWM a 0 por seguridad
        analogWrite(pinRPWM, 0);
        analogWrite(pinLPWM, 0);
        // Deshabilitar completamente el driver
        digitalWrite(pinR_EN, LOW);
        digitalWrite(pinL_EN, LOW);
    }

    /**
     * @brief Reinicia el contador de ticks del encoder a cero.
     */
    void resetEncoderValues() {
        encoder.write(0);
        posicionEncoder = 0;
    }

    // --- Métodos Getters para obtener información del estado ---

    /** @return La velocidad actual del motor en ticks por segundo. */
    float getVelocidadTicksPorSegundo() { return velocidadActual; }

    /** @return La velocidad actual del motor en Revoluciones Por Segundo (RPS). */
    float getVelocidadRPS() { return velocidadActual / pulsosPorRevolucion; }

    /** @return La velocidad actual del motor en Revoluciones Por Minuto (RPM). */
    float getVelocidadRPM() { return (velocidadActual / pulsosPorRevolucion) * 60.0f; }

    /** @return El último valor de salida del PID calculado (el valor PWM). */
    float getValorPWM() { return valorPWM; }
    
    void sincronizarRampa() {
    rampa.setVelocidadActual(velocidadActual);}

};

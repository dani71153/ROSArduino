#include <Arduino.h>

/**
 * @class PIDController
 * @brief Una clase genérica para implementar un controlador Proporcional-Integral-Derivativo.
 *
 * @details
 * Esta clase puede ser utilizada para cualquier sistema que requiera control de bucle cerrado,
 * como control de velocidad de motores, control de temperatura, mantenimiento de posición, etc.
 */
class PIDController {
private:
    float kp, ki, kd;          // Ganancias del PID
    float outMin, outMax;      // Límites de la salida (para saturación y anti-windup)
    
    float errorSum;            // Acumulador para el término Integral
    float lastInput;           // Último valor de entrada para el término Derivativo
    unsigned long lastTime;    // Último tiempo de ejecución para un cálculo de tiempo delta consistente
    unsigned long sampleTime;  // Intervalo de tiempo entre cálculos

public:
    /**
     * @brief Constructor del controlador PID.
     * @param Kp Ganancia Proporcional.
     * @param Ki Ganancia Integral.
     * @param Kd Ganancia Derivativa.
     * @param sampleTime_ms El intervalo de tiempo de muestreo en milisegundos.
     */
    PIDController(float Kp, float Ki, float Kd, unsigned long sampleTime_ms) {
        sampleTime = sampleTime_ms;
        setOutputLimits(-255, 255); // Límites por defecto para PWM de Arduino
        setTunings(Kp, Ki, Kd);
        lastTime = millis();
        errorSum = 0;
        lastInput = 0;
    }

    /**
     * @brief Calcula la salida del PID. Debe ser llamado regularmente.
     * @param setpoint El valor deseado (objetivo).
     * @param input El valor medido actualmente.
     * @return El valor de control calculado.
     */
    float compute(float setpoint, float input) {
        unsigned long now = millis();
        unsigned long timeChange = (now - lastTime);

        if (timeChange >= sampleTime) {
            // Calcular el error
            float error = setpoint - input;

            // Término Proporcional
            float p_term = kp * error;

            // Término Integral con anti-windup
            errorSum += (ki * error);
            errorSum = constrain(errorSum, outMin, outMax);
            float i_term = errorSum;

            // Término Derivativo
            float dInput = (input - lastInput);
            float d_term = -kd * dInput; // Se usa -kd para evitar "derivative kick"

            // Calcular la salida total
            float output = p_term + i_term + d_term;
            output = constrain(output, outMin, outMax);

            // Guardar estado para la siguiente iteración
            lastInput = input;
            lastTime = now;
            
            return output;
        }
        // Si no ha pasado el tiempo de muestreo, no se debería llamar, pero por seguridad retornamos un valor nulo.
        // En una implementación real, la lógica del loop previene esto.
        return NAN; // Not a Number, para indicar que no hubo cálculo.
    }

    /**
     * @brief Permite ajustar las ganancias del PID en tiempo de ejecución.
     */
    void setTunings(float Kp, float Ki, float Kd) {
        // Ajustar las ganancias al tiempo de muestreo para que sean más intuitivas
        float sampleTimeInSec = (float)sampleTime / 1000.0;
        kp = Kp;
        ki = Ki * sampleTimeInSec;
        kd = Kd / sampleTimeInSec;
    }

    /**
     * @brief Establece los límites de la salida del PID.
     */
    void setOutputLimits(float min, float max) {
        outMin = min;
        outMax = max;
    }

    /**
     * @brief Reinicia el estado del controlador (útil al cambiar de modo).
     */
    void reset() {
        errorSum = 0;
        lastInput = 0;
        lastTime = millis();
    }
    
};

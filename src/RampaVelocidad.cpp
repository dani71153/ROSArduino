#include <Arduino.h>

class RampaVelocidad {
private:
    float velocidadActual;
    float maxAceleracion;
    unsigned long tiempoPrevio;

    bool enFrenado = false;
    float referenciaPendiente = 0; // Nueva referencia almacenada

public:
    RampaVelocidad(float aceleracionMax)
        : velocidadActual(0), maxAceleracion(aceleracionMax), tiempoPrevio(millis()) {}

    void reiniciar() {
        velocidadActual = 0;
        tiempoPrevio = millis();
        enFrenado = false;
        referenciaPendiente = 0;
    }

    void setVelocidadActual(float nuevaVelocidad) {
        velocidadActual = nuevaVelocidad;
    }

    float getVelocidadActual() {
        return velocidadActual;
    }

    float actualizar(float referencia) {
        unsigned long tAhora = millis();
        float dt = (tAhora - tiempoPrevio) / 1000.0f;
        tiempoPrevio = tAhora;

        float pasoMax = maxAceleracion * dt;

        // Detectar cambio de sentido solo si estamos en movimiento
        bool hayCambioDeSigno = (velocidadActual > 0 && referencia < 0) ||
                                (velocidadActual < 0 && referencia > 0);

        if (!enFrenado && hayCambioDeSigno && fabs(velocidadActual) > 1e-2) {
            enFrenado = true;
            referenciaPendiente = referencia;
            referencia = 0; // Forzar frenado
            Serial.println("🛑 Entrando en fase de frenado");
        }

        if (enFrenado && fabs(velocidadActual) < 1.0) {
            enFrenado = false;
            referencia = referenciaPendiente;
            Serial.println("✅ Cambio de sentido autorizado");
        }

        // Movimiento normal (con o sin frenado)
        float delta = referencia - velocidadActual;
        if (fabs(delta) > pasoMax) {
            velocidadActual += copysign(pasoMax, delta);
        } else {
            velocidadActual = referencia;
        }

        return velocidadActual;
    }
};

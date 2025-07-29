#include <Arduino.h>

class RampaVelocidad {
private:
    float velocidadActual;
    float maxAceleracion;
    unsigned long tiempoPrevio;

public:
    RampaVelocidad(float aceleracionMax)
        : velocidadActual(0), maxAceleracion(aceleracionMax), tiempoPrevio(millis()) {}

    void reiniciar() {
        velocidadActual = 0;
        tiempoPrevio = millis();
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

        float delta = referencia - velocidadActual;
        float pasoMax = maxAceleracion * dt;

        if (fabs(delta) > pasoMax) {
            velocidadActual += copysign(pasoMax, delta);
        } else {
            velocidadActual = referencia;
        }

        return velocidadActual;
    }
};

#ifndef ODOMETRIA_H
#define ODOMETRIA_H

#include <Arduino.h>

// Estructura para almacenar la pose (posición y orientación)
struct Pose {
    float x;      // Posición X
    float y;      // Posición Y
    float theta;  // Orientación (rad)
};

class Odometria {
private:
    // Constantes del robot
    float ancho_eje;        // Distancia entre ruedas (L)
    float radio_rueda;      // Radio de las ruedas (R)
    int   ticks_por_rev;    // Pulsos por revolución del encoder

    // Estado actual
    Pose pose_actual;       // Pose actual del robot
    float v_izq, v_der;     // Velocidades actuales de ruedas (m/s)
    float v, w;             // Velocidad lineal y angular

    // Historial de posiciones
    Pose historial[10];
    int historial_idx;

    // Marco de referencia ('global' o 'local')
    bool marco_global;      // true: global, false: local

public:
    // Constructor
    Odometria(float ancho_eje, float radio_rueda, int ticks_por_rev);

    // Configuración y consulta de constantes
    void setConstantes(float ancho_eje, float radio_rueda, int ticks_por_rev);
    void setConstante(String nombre, float valor);
    void setConstante(String nombre, int valor);
    void getConstantes();
    float getConstante(String nombre);

    // Cinemática directa e inversa
    void calcularCinematicaDirecta(float v_izq, float v_der, float &v, float &w);
    void calcularCinematicaInversa(float v, float w, float &v_izq, float &v_der);

    // Odometría y actualización de pose con INCREMENTOS de ticks
    void actualizarConEncoders(long delta_ticks_izq, long delta_ticks_der, float dt);
    Pose getPose(bool global=true);
    void setPose(float x, float y, float theta);
    void resetearPose();

    // Historial
    void getHistorial(Pose* buffer, int &count);

    // Cambio de marco de referencia
    void setMarcoGlobal(bool global);

private:
    void guardarEnHistorial(float x, float y, float theta);
    void transformarPose(bool a_global);
};

#endif // ODOMETRIA_H

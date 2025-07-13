#include "odometria.h"

// Constructor
Odometria::Odometria(float ancho_eje, float radio_rueda, int ticks_por_rev) {
    this->ancho_eje = ancho_eje;  
    this->radio_rueda = radio_rueda;
    this->ticks_por_rev = ticks_por_rev;
    this->pose_actual = {0.0, 0.0, 0.0};
    this->v_izq = 0.0;
    this->v_der = 0.0;
    this->v = 0.0;
    this->w = 0.0;
    this->historial_idx = 0;
    this->marco_global = true;
    // Inicializar historial
    for (int i = 0; i < 10; i++) historial[i] = {0.0, 0.0, 0.0};
}

// Configuración y consulta de constantes

void Odometria::setConstante(String nombre, float valor) {
    if (nombre.equals("ancho_eje")) ancho_eje = valor;
    if (nombre.equals("radio_rueda")) radio_rueda = valor;
}

void Odometria::setConstante(String nombre, int valor) {
    if (nombre.equals("ticks_por_rev")) ticks_por_rev = valor;
}

void Odometria::getConstantes() {
    Serial.print("ancho_eje: "); Serial.println(ancho_eje, 4);
    Serial.print("radio_rueda: "); Serial.println(radio_rueda, 4);
    Serial.print("ticks_por_rev: "); Serial.println(ticks_por_rev);
}

float Odometria::getConstante(String nombre) {
    if (nombre.equals("ancho_eje")) return ancho_eje;
    if (nombre.equals("radio_rueda")) return radio_rueda;
    if (nombre.equals("ticks_por_rev")) return (float)ticks_por_rev;
    return 0.0;}

    
// Cinemática directa: v, w
void Odometria::calcularCinematicaDirecta(float v_izq, float v_der, float &v, float &w) {
    v = (v_der + v_izq) / 2.0;
    w = (v_der - v_izq) / ancho_eje;
}

// Cinemática inversa: v_izq, v_der
void Odometria::calcularCinematicaInversa(float v, float w, float &v_izq, float &v_der) {
    v_izq = v - (w * ancho_eje / 2.0);
    v_der = v + (w * ancho_eje / 2.0);
}

// Odometría y actualización de pose usando INCREMENTOS de ticks
void Odometria::actualizarConEncoders(long delta_ticks_izq, long delta_ticks_der, float dt) {
    if (dt <= 0.0) return; // Evitar división por cero

    // Calcular distancia recorrida por cada rueda (usando DELTAS)
    float vueltas_izq = (float)delta_ticks_izq / ticks_por_rev;
    float vueltas_der = (float)delta_ticks_der / ticks_por_rev;
    float dist_izq = vueltas_izq * 2 * PI * radio_rueda;
    float dist_der = vueltas_der * 2 * PI * radio_rueda;

    // Calcular velocidades lineales de ruedas
    v_izq = dist_izq / dt;
    v_der = dist_der / dt;

    // Cinemática directa
    calcularCinematicaDirecta(v_izq, v_der, v, w);

    // Calcular incremento de posición y orientación
    float delta_theta = w * dt;
    float delta_s = v * dt;

    // Actualizar orientación
    pose_actual.theta += delta_theta;
    if (pose_actual.theta > PI) pose_actual.theta -= 2 * PI;
    if (pose_actual.theta < -PI) pose_actual.theta += 2 * PI;

    // Actualizar posición
    pose_actual.x += delta_s * cos(pose_actual.theta);
    pose_actual.y += delta_s * sin(pose_actual.theta);

    guardarEnHistorial(pose_actual.x, pose_actual.y, pose_actual.theta);
}

// Devuelve la pose actual
Pose Odometria::getPose(bool global) {
    // En este ejemplo solo se maneja marco global
    // Si quieres transformar a marco local, implementa transformarPose()
    return pose_actual;
}

// Establece una nueva pose
void Odometria::setPose(float x, float y, float theta) {
    pose_actual.x = x;
    pose_actual.y = y;
    pose_actual.theta = theta;
    guardarEnHistorial(x, y, theta);
}

// Resetea la pose a (0,0,0)
void Odometria::resetearPose() {
    setPose(0.0, 0.0, 0.0);
}

// Historial de poses (devuelve una copia del buffer y el número de poses)
void Odometria::getHistorial(Pose* buffer, int &count) {
    count = 10;
    for (int i = 0; i < 10; i++) {
        buffer[i] = historial[i];
    }
}

// Cambio de marco (por ahora solo cambia el flag, no hace transformación)
void Odometria::setMarcoGlobal(bool global) {
    marco_global = global;
}

// Guardar nueva posición en historial
void Odometria::guardarEnHistorial(float x, float y, float theta) {
    historial[historial_idx] = {x, y, theta};
    historial_idx = (historial_idx + 1) % 10;
}

// (Opcional) transformar la pose (no implementado)
void Odometria::transformarPose(bool a_global) {
    // Implementar si usas otro marco de referencia
}

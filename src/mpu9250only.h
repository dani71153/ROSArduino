#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>

#ifndef TCAADDR
#define TCAADDR 0x70
#endif

struct MpuReading {
  bool  valid = false;
  float ax=0, ay=0, az=0;
  float gx=0, gy=0, gz=0;
  float mx=0, my=0, mz=0;
  float tempC = NAN;
};

// Inicializa I2C, TCA canal 0 y el MPU9250
void sensores_init();

// (Opcional si lo quieres reutilizar)
void tcaselect(uint8_t ch);
void tcadeselectAll();

// Lee el MPU9250 (canal TCA=0). Si emitSerial=true imprime A/G/M/T.
bool read_mpu(MpuReading* out = nullptr, bool emitSerial = false);

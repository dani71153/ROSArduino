#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BNO055.h>

// Configuración de multiplexor I2C y sensor
#define TCAADDR 0x70
#define I2C_KHZ 400000

void tcaselect(uint8_t i);
void tcadeselectAll();
bool rd8(uint8_t addr,uint8_t reg,uint8_t& val);
bool wr8(uint8_t addr,uint8_t reg,uint8_t val);
void mpu_quarantine(uint8_t a);
void read_mpu_on_channel(uint8_t ch);
void read_bno_on_channel(uint8_t ch);
void sensores_init();

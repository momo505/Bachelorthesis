#ifndef SAMPLE_H
#define SAMPLE_H
#include"Arduino.h"

// ========================= I2C HELPERS =========================
void writeI2C(byte addr, byte reg, byte data);
void setMotorSpeed(byte address, int speed);
void setMotorDirection(byte address, byte direction);

#endif

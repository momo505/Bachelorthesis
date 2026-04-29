#ifndef SAMPLE_H
#define SAMPLE_H
#include"Arduino.h"
#include <Wire.h>

// ========================= I2C HELPERS =========================
/*void writeI2C(byte addr, byte reg, byte data);
void setMotorSpeed(byte address, int speed);
void setMotorDirection(byte address, byte direction);
//*/
bool wireCheckAdressPresent(TwoWire* inter, byte addr, byte safereg);
//uint32_t wireCheckPresence();
bool wireCheckPresence(TwoWire &bus, byte addr);
#endif

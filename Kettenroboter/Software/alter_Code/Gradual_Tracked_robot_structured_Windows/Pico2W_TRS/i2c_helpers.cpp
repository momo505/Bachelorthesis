#include "i2c_helpers.h"
#include "Arduino.h"
#include <Wire.h>
/*
// ========================= I2C HELPERS =========================
void writeI2C(byte addr, byte reg, byte data) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}
void setMotorSpeed(byte address, int speed) {
    Wire.beginTransmission(address);
    Wire.write(SPEED_REG);
    Wire.write(constrain(speed, 0, 255));
    Wire.endTransmission();
}
void setMotorDirection(byte address, byte direction) {
    Wire.beginTransmission(address);
    Wire.write(DIR_REG);
    Wire.write(direction);
    Wire.endTransmission();
}
*/
// ========================= I2C HELPERS =========================
void writeI2C(byte addr, byte reg, byte data) {}
void setMotorSpeed(byte address, int speed) {}
void setMotorDirection(byte address, byte direction) {}

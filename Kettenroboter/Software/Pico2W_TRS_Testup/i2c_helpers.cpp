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
/*void writeI2C(byte addr, byte reg, byte data) {}
void setMotorSpeed(byte address, int speed) {}
void setMotorDirection(byte address, byte direction) {}
//*/

bool wireCheckAdressPresent(TwoWire* inter, byte addr, byte safereg){
    inter->beginTransmission(addr);
    inter->write(safereg);
    if(inter->endTransmission(true) == 0){
        inter->requestFrom(addr, 1);
        return true;
    };
    return false;
}

bool wireCheckPresence(TwoWire &Bus, byte addr){
    Bus.beginTransmission(addr);
    // 0 bedeutet Erfolg (ACK), alles andere ist ein Fehler (NACK) -> mehrere mögliche Fehlercodes
    byte error = Bus.endTransmission(addr);
    if(error == 0){
        return true;
    }else{
        return false;
    }
    // oder return (error == 0);
}

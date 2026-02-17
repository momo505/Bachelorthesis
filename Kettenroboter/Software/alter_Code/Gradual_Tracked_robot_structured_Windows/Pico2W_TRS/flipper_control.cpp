#include "flipper_control.h"
#include "Arduino.h"
#include "md03_config.h"
#include "motor_config.h"
#include "i2c_helpers.h"
// ========================= FLIPPER CONTROL =========================

volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;
volatile long encoderCount1B = 0;
volatile long encoderCount2B = 0;

long targetAngle = 0;
bool homed = false;
const float countsPerDegree = 24600.0 / 360.0; // 68,3333333333333
long homePosition1 = 0;
long homePosition2 = 0;

void moveForward() {
    digitalWrite(brakePin, HIGH);
    digitalWrite(brakePin2, HIGH);
    writeI2C(MD03_ADDR, SPEED_REG, 50);
    writeI2C(MD03_ADDR2, SPEED_REG, 50);
    delay(5);
    writeI2C(MD03_ADDR, CMD_REG, CMD_FORWARD);
    writeI2C(MD03_ADDR2, CMD_REG, CMD_FORWARD);
    delay(5);
}

void moveBackward() {
    digitalWrite(brakePin, HIGH);
    digitalWrite(brakePin2, HIGH);
    writeI2C(MD03_ADDR, SPEED_REG, 50);
    writeI2C(MD03_ADDR2, SPEED_REG, 50);
    delay(5); 
    writeI2C(MD03_ADDR, CMD_REG, CMD_REVERSE);
    writeI2C(MD03_ADDR2, CMD_REG, CMD_REVERSE);
    delay(5);
}

void stopMotor() {
    writeI2C(MD03_ADDR, SPEED_REG, 0);
    writeI2C(MD03_ADDR2, SPEED_REG, 0);
    delay(5);
    writeI2C(MD03_ADDR, CMD_REG, CMD_FORWARD);
    writeI2C(MD03_ADDR2, CMD_REG, CMD_FORWARD);
}

void releaseBrake() {
    digitalWrite(brakePin, HIGH);
    digitalWrite(brakePin2, HIGH);
    delay(50);
}

void applyBrake() {
    digitalWrite(brakePin, LOW);
    digitalWrite(brakePin2, LOW);
}

void homeMotor() {
    releaseBrake();
    moveForward();
    while (true) {
        if (digitalRead(hallSensorPin) == LOW && digitalRead(hallSensorPin2) == LOW) {
            stopMotor();
            encoderCount1 = 0; encoderCount2 = 0; encoderCount1B = 0; encoderCount2B = 0;
            homed = true;
            homePosition1 = encoderCount1;
            homePosition2 = encoderCount2;
            break;
        }
    }
    applyBrake();
}

void moveToAngle(long angle) {
    long targetAngle1 = (angle == -1) ? homePosition1 : encoderCount1 + angle * countsPerDegree;
    long targetAngle2 = (angle == -1) ? homePosition2 : encoderCount2 + angle * countsPerDegree;
    if (targetAngle1 > encoderCount1 && targetAngle2 > encoderCount2) moveForward();
    else moveBackward();
    while ((targetAngle1 > encoderCount1 && targetAngle2 > encoderCount2) || (targetAngle1 < encoderCount1 && targetAngle2 < encoderCount2));
    stopMotor();
    applyBrake();
}

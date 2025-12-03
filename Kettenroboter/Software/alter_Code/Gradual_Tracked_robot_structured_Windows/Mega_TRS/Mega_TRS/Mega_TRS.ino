#include <Wire.h>
//#include <SoftwareWire.h>
#include "MySoftwareWire.h"
//using namespace MySoftwareWire;
#include "mylibrary.h"

// ========================= MOTOR A/B CONFIG =========================
#define MOTOR_A_ADDR 0x5A
#define MOTOR_B_ADDR 0x5B
#define DIR_REG 0x00
#define SPEED_REG 0x02

#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 3

#define CPR 23400.0
#define WHEEL_DIAMETER 0.105
#define WHEEL_CIRCUMFERENCE (3.1416 * WHEEL_DIAMETER)

// PID parameters
float Kp = 2000;
float Ki = 1700;
float Kd = 300;

float setSpeed = 0.0;
bool speedSetByUser = false;

volatile long countA = 0, countB = 0;
long lastCountA = 0, lastCountB = 0;
unsigned long lastTime = 0;

// PID errors and integrals
float errorA = 0, prevErrorA = 0, integralA = 0;
float errorB = 0, prevErrorB = 0, integralB = 0;
float intMin = -1000, intMax = 1000;

// Previous PWM for ramping
int lastPwmA = 0, lastPwmB = 0;
int maxStep = 243;

char command = 's';
bool safeMode = false;  // stop motors if obstacle detected
bool moving = false;    // true only when movement command is given
bool firstStep = true;  // for smart ramping

// ========================= MD03 CONFIG flipper 1 =========================
#define MD03_ADDR 0x59
#define encoder_ChA 19
#define encoder_ChB 17
#define hallSensorPin 52
#define brakePin 7
#define CMD_REG 0x00
#define CMD_FORWARD 0x01
#define CMD_REVERSE 0x02

// ========================= MD03 CONFIG flipper 2 =========================
#define MD03_ADDR2 0x58
#define encoder_ChA2 18
#define encoder_ChB2 16
#define hallSensorPin2 50
#define brakePin2 6

volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;
volatile long encoderCount1B = 0;
volatile long encoderCount2B = 0;

long targetAngle = 0;
bool homed = false;
const float countsPerDegree = 24600.0 / 360.0;
long homePosition1 = 0;
long homePosition2 = 0;

// ========================= SOFTWARE I2C SRF02 =========================
MySoftwareWire sw(10, 11);
byte frontSensors[] = {0x78, 0x76, 0x7E};

// ========================= ENCODER INTERRUPTS =========================
void updateEncoder1() { if (digitalRead(encoder_ChB)) encoderCount1++; else encoderCount1--; }
void updateEncoder1B() { if (digitalRead(encoder_ChA)) encoderCount1--; else encoderCount1++; }
void updateEncoder2() { if (digitalRead(encoder_ChB2)) encoderCount2++; else encoderCount2--; }
void updateEncoder2B() { if (digitalRead(encoder_ChA2)) encoderCount2--; else encoderCount2++; }
void countEncoderA() { countA++; }
void countEncoderB() { countB++; }

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

// ========================= PID RESET =========================
void resetPIDandEncoders() {
    countA = 0;
    countB = 0;
    lastCountA = 0;
    lastCountB = 0;
    integralA = 0;
    integralB = 0;
    prevErrorA = 0;
    prevErrorB = 0;
    lastPwmA = 0;
    lastPwmB = 0;
    firstStep = true;
    lastTime = millis();
}

// ========================= MOTOR COMMANDS =========================
void handleCommand(char cmd) {
    switch (cmd) {
        case 'f': setMotorDirection(MOTOR_A_ADDR, 1); setMotorDirection(MOTOR_B_ADDR, 1); break;
        case 'b': setMotorDirection(MOTOR_A_ADDR, 2); setMotorDirection(MOTOR_B_ADDR, 2); break;
        case 'l': setMotorDirection(MOTOR_A_ADDR, 2); setMotorDirection(MOTOR_B_ADDR, 1); break;
        case 'r': setMotorDirection(MOTOR_A_ADDR, 1); setMotorDirection(MOTOR_B_ADDR, 2); break;
        case 's': setMotorDirection(MOTOR_A_ADDR, 0); setMotorDirection(MOTOR_B_ADDR, 0); break;
    }
    resetPIDandEncoders();
}

// ========================= FLIPPER CONTROL =========================
void moveForward() { digitalWrite(brakePin, HIGH); digitalWrite(brakePin2, HIGH); writeI2C(MD03_ADDR, SPEED_REG, 50); writeI2C(MD03_ADDR2, SPEED_REG, 50); delay(5); writeI2C(MD03_ADDR, CMD_REG, CMD_FORWARD); writeI2C(MD03_ADDR2, CMD_REG, CMD_FORWARD); delay(5); }
void moveBackward() { digitalWrite(brakePin, HIGH); digitalWrite(brakePin2, HIGH); writeI2C(MD03_ADDR, SPEED_REG, 50); writeI2C(MD03_ADDR2, SPEED_REG, 50); delay(5); writeI2C(MD03_ADDR, CMD_REG, CMD_REVERSE); writeI2C(MD03_ADDR2, CMD_REG, CMD_REVERSE); delay(5); }
void stopMotor() { writeI2C(MD03_ADDR, SPEED_REG, 0); writeI2C(MD03_ADDR2, SPEED_REG, 0); delay(5); writeI2C(MD03_ADDR, CMD_REG, CMD_FORWARD); writeI2C(MD03_ADDR2, CMD_REG, CMD_FORWARD); }
void releaseBrake() { digitalWrite(brakePin, HIGH); digitalWrite(brakePin2, HIGH); delay(50); }
void applyBrake() { digitalWrite(brakePin, LOW); digitalWrite(brakePin2, LOW); }
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
    stopMotor(); applyBrake();
}

// ========================= SRF02 READ =========================
int readSRF02(byte address) {
    sw.beginTransmission(address);
    sw.write(0x00); sw.write(0x51); sw.endTransmission();
    delay(70);
    sw.beginTransmission(address); sw.write(0x02); sw.endTransmission();
    sw.requestFrom((int)address, 2);
    if (sw.available() >= 2) return (sw.read() << 8) + sw.read();
    return -1;
}

// ========================= SETUP =========================
void setup() {
    int test;
    test = example(2, 'k');
    delay((unsigned long)test);
    Serial.begin(115200);
    Wire.begin();
    sw.begin();

    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);
    pinMode(encoder_ChA, INPUT_PULLUP); pinMode(encoder_ChB, INPUT_PULLUP);
    pinMode(encoder_ChA2, INPUT_PULLUP); pinMode(encoder_ChB2, INPUT_PULLUP);
    pinMode(hallSensorPin, INPUT_PULLUP); pinMode(hallSensorPin2, INPUT_PULLUP);
    pinMode(brakePin, OUTPUT); pinMode(brakePin2, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), countEncoderA, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), countEncoderB, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder_ChA), updateEncoder1, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder_ChB), updateEncoder1B, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder_ChA2), updateEncoder2, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder_ChB2), updateEncoder2B, RISING);

    resetPIDandEncoders();
    handleCommand(command);
    releaseBrake();
    homeMotor();
    applyBrake();

    if (homed) Serial.println(" Homing done! Enter speed like this → speed:0.05");
}

// ========================= LOOP =========================
void loop() {
    // --------- CHECK FRONT OBSTACLES ---------
    int sum = 0, valid = 0;
    for (int i = 0; i < 3; i++) { int d = readSRF02(frontSensors[i]); if (d > 0) { sum += d; valid++; } }
    int avgFront = (valid > 0) ? (sum / valid) : 999;

    if (avgFront < 40 && !safeMode) {
        setMotorSpeed(MOTOR_A_ADDR, 0); setMotorSpeed(MOTOR_B_ADDR, 0);
        setMotorDirection(MOTOR_A_ADDR, 0); setMotorDirection(MOTOR_B_ADDR, 0);
        command = 's'; safeMode = true; moving = false;
        resetPIDandEncoders();

        Serial.println(" Obstacle detected! Enter flipper angle like this → flipperAngle:30 or flipperAngle:home (0 = ignore):");
        bool flipperDone = false;
        while (!flipperDone) {
            if (Serial.available()) {
                String input = Serial.readStringUntil('\n'); input.trim();
                if (input.startsWith("flipperAngle:")) {
                    String val = input.substring(input.indexOf(':') + 1);
                    if (val == "home") moveToAngle(-1);
                    else { long angle = val.toInt(); if (angle != 0) moveToAngle(angle); }
                    applyBrake(); flipperDone = true;
                    Serial.println(" Flipper moved. Waiting for user command...");
                }
            }
        }
    }

    // --------- SERIAL COMMANDS ---------
    while (Serial.available()) {
        String input = Serial.readStringUntil('\n'); input.trim();
        if (input.length() == 1) { command = input.charAt(0); handleCommand(command); moving = (command != 's'); safeMode = false; }
        else if (input.startsWith("speed:") && homed) { setSpeed = input.substring(input.indexOf(':') + 1).toFloat(); speedSetByUser = true; Serial.print("✅ New set speed: "); Serial.println(setSpeed); }
        else if (input.startsWith("flipperAngle:")) { String val = input.substring(input.indexOf(':') + 1); if (val == "home") moveToAngle(-1); else moveToAngle(val.toInt()); }
        while (Serial.available()) Serial.read();
    }

    // --------- PID CONTROL ---------
    if (!safeMode && speedSetByUser && moving) {
        unsigned long now = millis(); float dt = (now - lastTime) / 1000.0;
        if (dt >= 0.05) {
            long deltaA = countA - lastCountA; long deltaB = countB - lastCountB;
            lastCountA = countA; lastCountB = countB;

            float speedA = (deltaA / CPR * WHEEL_CIRCUMFERENCE) / dt;
            float speedB = (deltaB / CPR * WHEEL_CIRCUMFERENCE) / dt;

            errorA = setSpeed - speedA; integralA += errorA * dt; integralA = constrain(integralA, intMin, intMax);
            float derivativeA = (errorA - prevErrorA) / dt; prevErrorA = errorA;
            float outputA = Kp * errorA + Ki * integralA + Kd * derivativeA;

            errorB = setSpeed - speedB; integralB += errorB * dt; integralB = constrain(integralB, intMin, intMax);
            float derivativeB = (errorB - prevErrorB) / dt; prevErrorB = errorB;
            float outputB = Kp * errorB + Ki * integralB + Kd * derivativeB;

            int pwmA = constrain(outputA, 0, 243); int pwmB = constrain(outputB, 0, 243);

            if (firstStep) { lastPwmA = pwmA; lastPwmB = pwmB; firstStep = false; }
            else { pwmA = constrain(pwmA, lastPwmA - maxStep, lastPwmA + maxStep); pwmB = constrain(pwmB, lastPwmB - maxStep, lastPwmB + maxStep); lastPwmA = pwmA; lastPwmB = pwmB; }

            setMotorSpeed(MOTOR_A_ADDR, pwmA); setMotorSpeed(MOTOR_B_ADDR, pwmB);

            Serial.print(setSpeed); Serial.print(" "); Serial.print(speedA); Serial.print(" "); Serial.println(speedB);

            lastTime = now;
        }
    }
}

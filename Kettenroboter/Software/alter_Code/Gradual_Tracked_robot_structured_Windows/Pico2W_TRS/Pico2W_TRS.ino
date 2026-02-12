#include <Wire.h>
#include <FreeRTOS.h>
#include "MySoftwareWire.h"

//compile with acli compile -j 0 -v -e -b rp2040:rp2040:rpipico2w
// find port: sudo dmesg | egrep -i --color 'ttyACM'
// arduino-cli upload I2Csketch.ino -p /dev/ttyACM0 -b rp2040:rp2040:rpipico2w
// serial: sudo screen /dev/ttyACM0 115200          (ctrl-d to detach)

// ========================= MOTOR A/B CONFIG =========================
#include "motor_config.h"

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
#include "md03_config.h"

/*
volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;
volatile long encoderCount1B = 0;
volatile long encoderCount2B = 0;
*/
/*
long targetAngle = 0;
bool homed = false;
const float countsPerDegree = 24600.0 / 360.0; // 68,3333333333333
long homePosition1 = 0;
long homePosition2 = 0;
*/
#define ENCODER_COUNTS_FULL_ROT 24600.0
#define COUNTS_PER_DEGREE (ENCODER_COUNTS_FULL_ROT / 360.0)

// ========================= SOFTWARE I2C SRF02 =========================
MySoftwareWire sw(10, 11);
byte frontSensors[] = {0x78, 0x76, 0x7E};

// ========================= FLIPPER CONTROL =========================
#include "flipper_control.h"


// ========================= ENCODER INTERRUPTS =========================
void updateEncoder1() { if (digitalRead(encoder_ChB)) encoderCount1++; else encoderCount1--; }
void updateEncoder1B() { if (digitalRead(encoder_ChA)) encoderCount1--; else encoderCount1++; }
void updateEncoder2() { if (digitalRead(encoder_ChB2)) encoderCount2++; else encoderCount2--; }
void updateEncoder2B() { if (digitalRead(encoder_ChA2)) encoderCount2--; else encoderCount2++; }
void countEncoderA() { countA++; }
void countEncoderB() { countB++; }

// ========================= I2C HELPERS =========================
#include "i2c_helpers.h"

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
        case 'f':
            setMotorDirection(MOTOR_A_ADDR, 1);
            setMotorDirection(MOTOR_B_ADDR, 1);
            break;
        case 'b':
            setMotorDirection(MOTOR_A_ADDR, 2);
            setMotorDirection(MOTOR_B_ADDR, 2);
            break;
        case 'l':
            setMotorDirection(MOTOR_A_ADDR, 2);
            setMotorDirection(MOTOR_B_ADDR, 1);
            break;
        case 'r':
            setMotorDirection(MOTOR_A_ADDR, 1);
            setMotorDirection(MOTOR_B_ADDR, 2);
            break;
        case 's':
            setMotorDirection(MOTOR_A_ADDR, 0);
            setMotorDirection(MOTOR_B_ADDR, 0);
            break;
    }
    resetPIDandEncoders();
}

// ========================= SRF02 READ =========================
#include "srf02.h"

// ========================= Postbox / Pico-SDK Mutex =========================
//include <FreeRTOS.h>
#include "semphr.h"
#define POSTBOXSIZE 32

SemaphoreHandle_t  pb_mutex = xSemaphoreCreateMutex();
volatile bool newcontent = false;
volatile uint32_t postbox[POSTBOXSIZE] = {0};

//pb_mutex = xSemaphoreCreateMutex();
/*
// ========================= MUTEX / POSTBOX =========================
SemaphoreHandle_t mutex = NULL;
mutex = xSemaphoreCreateMutex();

volatile uint32_t postbox[32];
volatile bool newcontent = False;
*/

// ========================= SETUP =========================
void setup() {
    Serial.begin(115200);
    Wire.begin();
    sw.begin();
    //Wire.SDA()
    //Wire.SCL()

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

    //Mutex für postbox
    if(pb_mutex != NULL){
        Serial.println("Mutex created postbox up and working ");
    }else{
        Serial.println("!Error: Could not create Mutex!");
    }
    if(xSemaphoreTake(pb_mutex, portMAX_DELAY)){
        delay(1);
        xSemaphoreGive(pb_mutex);
    }


}

void setup1(){

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
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() == 1) {
            command = input.charAt(0); handleCommand(command);
            moving = (command != 's'); safeMode = false;
        }
        else if (input.startsWith("speed:") && homed) {
            setSpeed = input.substring(input.indexOf(':') + 1).toFloat();
            speedSetByUser = true;
            Serial.print(" New set speed: "); Serial.println(setSpeed);
        }
        else if (input.startsWith("flipperAngle:")) {
            String val = input.substring(input.indexOf(':') + 1);
            if (val == "home") moveToAngle(-1);
            else moveToAngle(val.toInt());
        }
        while (Serial.available()) Serial.read();
    }

    // --------- PID CONTROL ---------
    if (!safeMode && speedSetByUser && moving) {
        unsigned long now = millis(); float dt = (now - lastTime) / 1000.0;
        if (dt >= 0.05) {
            long deltaA = countA - lastCountA;
            long deltaB = countB - lastCountB;
            lastCountA = countA;
            lastCountB = countB;

            float speedA = (deltaA / CPR * WHEEL_CIRCUMFERENCE) / dt;
            float speedB = (deltaB / CPR * WHEEL_CIRCUMFERENCE) / dt;

            errorA = setSpeed - speedA; integralA += errorA * dt;
            integralA = constrain(integralA, intMin, intMax);
            float derivativeA = (errorA - prevErrorA) / dt;
            prevErrorA = errorA;
            float outputA = Kp * errorA + Ki * integralA + Kd * derivativeA;

            errorB = setSpeed - speedB; integralB += errorB * dt;
            integralB = constrain(integralB, intMin, intMax);
            float derivativeB = (errorB - prevErrorB) / dt;
            prevErrorB = errorB;
            float outputB = Kp * errorB + Ki * integralB + Kd * derivativeB;

            int pwmA = constrain(outputA, 0, 243);
            int pwmB = constrain(outputB, 0, 243);

            if (firstStep) {
                lastPwmA = pwmA;
                lastPwmB = pwmB;
                firstStep = false;
            }
            else {
                pwmA = constrain(pwmA, lastPwmA - maxStep, lastPwmA + maxStep);
                pwmB = constrain(pwmB, lastPwmB - maxStep, lastPwmB + maxStep);
                lastPwmA = pwmA; lastPwmB = pwmB;
            }

            setMotorSpeed(MOTOR_A_ADDR, pwmA);
            setMotorSpeed(MOTOR_B_ADDR, pwmB);

            Serial.print(setSpeed); Serial.print(" "); Serial.print(speedA); Serial.print(" "); Serial.println(speedB);

            lastTime = now;
        }
    }
    // --------- Postbox data exchange ---------
    if(xSemaphoreTake(pb_mutex, portMAX_DELAY)){
        Serial.println("Postbox open to Core0, owning Mutex");
        Serial.print("   newcontent:"); if(newcontent==true){Serial.println("TRUE");} else{Serial.println("FALSE");}
        for(int i=0; i<POSTBOXSIZE; i++){
            postbox[i] = i;
        }
        newcontent = true;
        Serial.println("Postbox filled returning Mutex");
        xSemaphoreGive(pb_mutex);
    }
}

void loop1(){
    if(xSemaphoreTake(pb_mutex, portMAX_DELAY)){
        if(newcontent == true){
            for(int i=0; i<POSTBOXSIZE; i++){
                postbox[i] = i;
            }
        }
        xSemaphoreGive(pb_mutex);
    }
}

#ifndef MOTOR_CONFIG
#define MOTOR_CONFIG

// ========================= MOTOR A/B CONFIG =========================
#define MOTOR_A_ADDR 0x5A
#define MOTOR_B_ADDR 0x5B

#define DIR_REG 0x00
#define STATUS_REG 0x01
#define SPEED_REG 0x02
#define ACCEL_REG 0x03
#define TEMP_REG 0x04
#define CURR_REG 0x05
#define VERSION_REG 0x07

//#define ENCODER_A_PIN 2
//#define ENCODER_B_PIN 3

#define CPR 23400.0
#define WHEEL_DIAMETER 0.105
#define WHEEL_CIRCUMFERENCE (3.1416 * WHEEL_DIAMETER)

#endif

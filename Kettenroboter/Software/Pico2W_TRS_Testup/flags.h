#ifndef CUSTOM_FLAGS
#define CUSTOM_FLAGS

enum message {forward, backward, left, right, stop, special};

#define C1_MOTOR_I2C_ERROR 2^0
// #define SENSOR_I2C_ERROR 2
#define C1_MOVEMENT_ALLOWED 2^1
#define C1_MOVEM
#define C1_EMERGENCY_STOP 2^31

#define FLAG31 31

#endif
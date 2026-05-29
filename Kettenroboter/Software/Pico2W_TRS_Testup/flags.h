#ifndef CUSTOM_FLAGS
#define CUSTOM_FLAGS

//enum message {forward, backward, left, right, stop, special};

#define C1_EMERGENCY_STOP  (1 << 0)   

#define C1_MOVEMENT_ALLOWED (1 << 8)
#define C1_DRIVE_MANUAL_MODE (1 << 9)
#define C1_FLIPPER_BREAK_ON (1 << 10)
#define C1_FLIPPER_MANUAL_MODE (1 << 11)

#define C1_CONTROLMODE (1 << 15) 
#define C1_GATHER_MOTORDATA (1 << 16)

#define C1_MOTOR_I2C_ERROR (1 << 24)

#endif
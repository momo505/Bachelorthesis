#ifndef FLIPPER_CONTROL
#define FLIPPER_CONTROL


extern volatile long encoderCount1;
extern volatile long encoderCount2;
extern volatile long encoderCount1B;
extern volatile long encoderCount2B;
extern long homePosition1;
extern long homePosition2;
extern bool homed;

// ========================= FLIPPER CONTROL =========================
void moveForward();
void moveBackward();
void stopMotor();
void releaseBrake();
void applyBrake();
void homeMotor();
void moveToAngle(long angle);

#endif

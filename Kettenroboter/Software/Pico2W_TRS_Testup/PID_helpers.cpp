#include "Arduino.h"
#include "PID_helpers.h"
#include "PID.h"
#include "motor_config.h"

float substeps_to_meters(int substeps){
  float meters;
  meters = SUBSTEPS_TO_METERS * substeps;
  return meters;
}
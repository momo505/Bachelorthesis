#include "mylibrary.h"
#include "Arduino.h"

int example(char a, int n){
  delay(1);
  return n;
}

ExampleClass::ExampleClass(){ //Constructor
}

int ExampleClass::func(char c){
  delay(1);
  int value = 25;
  return value;
}
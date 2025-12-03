#include "MySoftwareWire.h"
#include <Wire.h>

#include "Arduino.h"

//using namespace MySoftwareWire //{

/*
MySoftwareWire::helloWorld(){
    Serial.println("Hello World!");
}
*/

// Konstruktor: Jetzt mit dem korrekten Klassennamen und korrekter Zuweisung der Membervariablen
MySoftwareWire::MySoftwareWire(int sdaPin, int sclPin) { 
	// sdaPin und sclPin sind die Parameter des Konstruktors
	// SDA und SCL sind die Membervariablen der Klasse (aus MySoftwareWire.h)
	SDA = sdaPin;
	SCL = sclPin;
}

// Rückgabetyp 'void' für beginTransmission und 'int' für read muss in der Implementierung enthalten sein, falls in .h definiert.
void MySoftwareWire::beginTransmission(int address){
				delay(1); //place holder
			}
void MySoftwareWire::write(byte adress){
				delay(1); //place holder
			}
void MySoftwareWire::endTransmission(){
				delay(1); //place holder
			}
void MySoftwareWire::requestFrom(byte adress, int number){
				delay(1); //place holder
			}
int MySoftwareWire::available(){
				delay(1); //place holder
				// Muss int zurückgeben, nicht boolean True (oder true)
				return 0; 
			}
int MySoftwareWire::read(){
				delay(1); //place holder
				// Muss int zurückgeben
				return 0;
			}
void MySoftwareWire::begin(){
				delay(1); //place holder
			}
//}// ending namespace
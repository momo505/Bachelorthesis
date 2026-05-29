/*
#include "my_serial.h"
#include "Arduino.h"

//MySerialUSB::MySerialUSB(uint8_t select){
MySerialUSB::MySerialUSB(uint8_t select) : core(select) {
	core = select;
}


void MySerialUSB::begin(unsigned long mybaud){
	Serial.begin(mybaud);
}

size_t MySerialUSB::print(const char output[]){
	//check
	if(rp2040.cpuid()==core){
		return Serial.print(output);
	}
	return 0;
}

size_t MySerialUSB::println(const char output[]){
	//check
	if(rp2040.cpuid()==core){
		return Serial.println(output);
	}
	return 0;
}

#include "my_serial.h"

MySerialUSB::MySerialUSB(uint8_t select) : core(select) {
	// Doppelte Zuweisung entfernt, da "core(select)" bereits initialisiert
}

void MySerialUSB::begin(unsigned long mybaud){
	Serial.begin(mybaud);
}

// Jedes print/println nutzt intern diese write-Methode
size_t MySerialUSB::write(uint8_t character){
	if(rp2040.cpuid() == core){
		return Serial.write(character);
	}
	return 0;
}
*/

#include "my_serial.h"

MySerialUSB::MySerialUSB(uint8_t select) : core(select) {}

void MySerialUSB::begin(unsigned long mybaud){
	Serial.begin(mybaud);
}

// Ausgabe
size_t MySerialUSB::write(uint8_t character){
	if(rp2040.cpuid() == core){
		return Serial.write(character);
	}
	return 0;
}

// Eingabe: Liefert nur Daten, wenn der aufrufende Core passt
int MySerialUSB::available(){
	if(rp2040.cpuid() == core){
		return Serial.available();
	}
	return 0;
}

int MySerialUSB::read(){
	if(rp2040.cpuid() == core){
		return Serial.read();
	}
	return -1; // Arduino-Standard für "keine Daten vorhanden"
}

int MySerialUSB::peek(){
	if(rp2040.cpuid() == core){
		return Serial.peek();
	}
	return -1;
}
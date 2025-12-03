#ifndef SWIRE_H
#define SWIRE_H

#include "Arduino.h"
	
    //void helloWorld();
	
	class MySoftwareWire{
		private:
			int SDA;
			int SCL;
		public:
			MySoftwareWire(int SDA, int SCL); //Constructor (Konstruktorname muss mit dem Klassennamen übereinstimmen)
			//int read := 0;
			void beginTransmission(int address);
			void write(byte adress);
			void endTransmission();
			void requestFrom(byte adress, int number);
			int available();
			int read();
			void begin();
		
	};
//}// ending namespace
#endif
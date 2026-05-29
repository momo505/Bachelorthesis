/*#ifndef SERIALCOMMS_H
#define SERIALCOMMS_H

#include <cstddef>  // for size_t
#include <cstdint>  // for uint8_t

class MySerialUSB{
	public:
		MySerialUSB(uint8_t select);
		void begin(unsigned long mybaud);
		size_t print(const char output[]);
		size_t println(const char output[]);
		uint8_t core;
};

#endif

#ifndef SERIALCOMMS_H
#define SERIALCOMMS_H

#include <cstdint>  // for uint8_t
#include "Arduino.h" // Nötig für die Print-Klasse

// Erbe von der Arduino-Klasse "Print"
class MySerialUSB : public Print {
	public:
		MySerialUSB(uint8_t select);
		void begin(unsigned long mybaud);
		
		// Die Print-Klasse benötigt nur diese virtuelle Methode von dir
		virtual size_t write(uint8_t character) override;
		
		uint8_t core;
};

#endif
*/
#ifndef SERIALCOMMS_H
#define SERIALCOMMS_H

#include <cstdint>  // for uint8_t
#include "Arduino.h" // Nötig für die Stream-Klasse

// Von "Stream" erben (beinhaltet bereits "Print")
class MySerialUSB : public Stream {
	public:
		MySerialUSB(uint8_t select);
		void begin(unsigned long mybaud);
		
		// Für die Ausgabe (von Print gefordert)
		virtual size_t write(uint8_t character) override;
		
		// Für die Eingabe (von Stream gefordert)
		virtual int available() override;
		virtual int read() override;
		virtual int peek() override;
		
		uint8_t core;
};

#endif
#include "srf02.h"
#include "Arduino.h"

// ========================= SRF02 READ =========================
int readSRF02(byte address) {
    //sw.beginTransmission(address);
    //sw.write(0x00); sw.write(0x51); sw.endTransmission();
    delay(70);
    //sw.beginTransmission(address); sw.write(0x02); sw.endTransmission();
    //sw.requestFrom((int)address, 2);
    //if (sw.available() >= 2) return (sw.read() << 8) + sw.read();
    return -1;
}


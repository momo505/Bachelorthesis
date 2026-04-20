#ifndef QEI_H
#define QEI_H

#include <hardware/pio.h>

// Our assembled program:
#include "qei.pio.h"

PIO pio = pio0;

void pio_setup(uint8_t pio_input_pins[]);

#endif

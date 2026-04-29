// pio_qei.cpp
#include "pio_qei.h"

#include "Arduino.h"
//#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
// Our assembled program:
//#include "blink.pio.h"
#include "qei.pio.h"

bool pio_qei_setup(int pin, float pio_freq) {


    // Choose PIO instance (0 or 1)
    PIO pio = pio0;

    // Get first free state machine in PIO 0
    int sm = pio_claim_unused_sm(pio, true);

    // Add PIO program to PIO instruction memory. SDK will find location and
    // return with the memory offset of the program.
    int offset = pio_add_program(pio, &qei_program);

    // Calculate the PIO clock divider
    float div = (float)clock_get_hz(clk_sys) / pio_freq;

    // Initialize the program using the helper function in our .pio file
    qei_init(pio, sm, offset, pin, div);

    // Start running our PIO program in the state machine
    pio_sm_set_enabled(pio, sm, true);
	
	//Return true if instantiation successfull
	if((sm >=  0) || (offset >=  0)){
		return true;
	}else{
    return false;
    }
}
    
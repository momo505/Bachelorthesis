
#include <hardware/pio.h>
#include "pio_qei.h"
// assembled program:
#include "qei.pio.h"

void pio_setup(uint8_t pio_input_pins[])
	uint offset = pio_add_program(pio, &qei_program);
	uint sm = pio_claim_unused_sm(pio, true) //not sure what true is for
	pio_qei_init(pio, sm, offset, SOME_PIN) //needs init function in qei.pio
}
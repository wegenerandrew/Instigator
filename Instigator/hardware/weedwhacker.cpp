#include "hardware/weedwhacker.h"

#include <avr/io.h>

static PORT_t &weedwhacker_port = PORTH;
static const int weedwhacker_mask = _BV(6);

void weedwhacker_init() {
	weedwhacker_port.OUTCLR = weedwhacker_mask;
	weedwhacker_port.DIRSET = weedwhacker_mask;
}

void weedwhacker_power(bool on) {
	if (on) {
		weedwhacker_port.OUTSET = weedwhacker_mask;
	} else {
		weedwhacker_port.OUTCLR = weedwhacker_mask;
	}
}

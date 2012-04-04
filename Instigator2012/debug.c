#include "debug.h"
#include <avr/io.h>

static const int errorled_mask = _BV(1);

void debug_init() {
	PORTR.DIRSET = errorled_mask;
	PORTR.OUTSET = errorled_mask;
}

void debug_setErrorLED() {
	PORTR.OUTCLR = errorled_mask;
}